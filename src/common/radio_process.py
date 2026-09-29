"""Radio in its own OS process.

With TCP radio links, a node that lost its radio froze every Zenoh session in
its process for ~10 s (Zenoh 1.0.4 and 1.10.1 alike); with the radio in the
same process as the onboard link, a jammed drone lost its own sensors for 10 s
and missed its engagement.  The radio now uses UDP, which avoids that freeze,
and this separation stays as defence in depth.  Here the radio session lives in a
child process and talks to the parent over a pipe, so a radio failure only
stops radio traffic, as with a separate radio module on a real drone.

RadioProcess offers the subset of the zenoh.Session API the agent and ship use:
``declare_publisher(key).put(data)``, ``declare_subscriber(key, callback)``
and ``close()``.  Callbacks get a sample with ``key_expr`` (str) and
``payload`` (bytes).  Sending never blocks the caller: messages go through a
bounded queue and the oldest are dropped if the radio process stalls.
"""

import logging
import multiprocessing as mp
import queue
import threading
from collections import namedtuple

log = logging.getLogger("RadioProcess")

Sample = namedtuple("Sample", ["key_expr", "payload"])


def _worker(conn, subnet, routing_mode, lease_ms):
    """Child process: owns the Zenoh radio session and relays over the pipe."""
    import links   # imported here so the child opens its own Zenoh runtime
    import os, sys, time
    debug = os.environ.get("RADIO_DEBUG") == "1"
    session = links.open_radio(subnet, routing_mode, lease_ms)
    if debug:
        print("[radio child] open_timeout", session.config().get_json("transport/unicast/open_timeout")
              if hasattr(session, "config") else "?", file=sys.stderr, flush=True)
    stats = {"put_max": 0.0, "last_rx": time.monotonic(), "rx_gap": 0.0, "t": time.monotonic()}
    publishers, subscribers = {}, []
    send_lock = threading.Lock()

    def forwarder(sub_id):
        def forward(sample):
            if debug and str(sample.key_expr) not in publishers:   # remote traffic only
                now = time.monotonic()
                stats["rx_gap"] = max(stats["rx_gap"], now - stats["last_rx"])
                stats["last_rx"] = now
            try:
                with send_lock:
                    conn.send(("msg", sub_id, str(sample.key_expr), bytes(sample.payload)))
            except (BrokenPipeError, EOFError, OSError):
                pass
        return forward

    while True:
        try:
            cmd = conn.recv()
        except (EOFError, OSError):
            break
        op = cmd[0]
        if op == "put":
            _, key, data = cmd
            pub = publishers.get(key)
            if pub is None:
                pub = publishers[key] = session.declare_publisher(key)
            t0 = time.monotonic()
            pub.put(data)
            if debug:
                stats["put_max"] = max(stats["put_max"], time.monotonic() - t0)
                if t0 - stats["t"] >= 1.0:
                    print(f"[radio child] put_max={stats['put_max']*1000:.1f}ms rx_gap={stats['rx_gap']*1000:.0f}ms",
                          file=sys.stderr, flush=True)
                    stats.update(put_max=0.0, rx_gap=0.0, t=t0)
        elif op == "sub":
            _, sub_id, key = cmd
            subscribers.append(session.declare_subscriber(key, forwarder(sub_id)))
        elif op == "close":
            break
    session.close()


class _Publisher:
    def __init__(self, radio, key):
        self._radio, self._key = radio, key

    def put(self, data):
        self._radio._send(("put", self._key, data if isinstance(data, (bytes, str)) else bytes(data)))

    def undeclare(self):
        pass


class _Subscriber:
    def undeclare(self):
        pass


class RadioProcess:
    SEND_QUEUE = 1000

    def __init__(self, subnet=None, routing_mode=None, lease_ms=None):
        ctx = mp.get_context("spawn")   # never fork a process that already runs a Zenoh runtime
        self._conn, child = ctx.Pipe()
        self._proc = ctx.Process(target=_worker, args=(child, subnet, routing_mode, lease_ms),
                                 name="radio", daemon=True)
        self._proc.start()
        child.close()
        self._callbacks = []            # indexed by subscription id
        self._out = queue.Queue(maxsize=self.SEND_QUEUE)
        self._dropped = 0
        self._running = True
        threading.Thread(target=self._sender, name="radio-tx", daemon=True).start()
        threading.Thread(target=self._receiver, name="radio-rx", daemon=True).start()

    # --- Session-like API ------------------------------------------------
    def declare_publisher(self, key):
        return _Publisher(self, key)

    def declare_subscriber(self, key, callback):
        self._callbacks.append(callback)
        self._send(("sub", len(self._callbacks) - 1, key))
        return _Subscriber()

    def put(self, key, data):
        self._send(("put", key, data))

    def close(self):
        self._running = False
        try:
            self._conn.send(("close",))
        except (BrokenPipeError, OSError):
            pass
        self._proc.join(timeout=3)
        if self._proc.is_alive():
            self._proc.terminate()

    @property
    def alive(self):
        return self._proc.is_alive()

    # --- plumbing ----------------------------------------------------------
    def _send(self, cmd):
        try:
            self._out.put_nowait(cmd)
        except queue.Full:
            # The radio process is stalled: drop the oldest message rather than block the caller.
            try:
                self._out.get_nowait()
            except queue.Empty:
                pass
            self._dropped += 1
            self._out.put_nowait(cmd)

    def _sender(self):
        while self._running:
            cmd = self._out.get()
            try:
                self._conn.send(cmd)
            except (BrokenPipeError, OSError):
                log.error("radio process is gone")
                return

    def _receiver(self):
        while self._running:
            try:
                op, sub_id, key, payload = self._conn.recv()
            except (EOFError, OSError):
                return
            if op != "msg":
                continue
            try:
                self._callbacks[sub_id](Sample(key, payload))
            except Exception as e:
                log.warning("radio callback error on %s: %s", key, e)
