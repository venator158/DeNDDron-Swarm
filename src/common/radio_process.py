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
import time
from collections import namedtuple

log = logging.getLogger("RadioProcess")

Sample = namedtuple("Sample", ["key_expr", "payload"])


def _worker(conn, subnet, routing_mode, lease_ms):
    """Child process: owns the Zenoh radio session and relays over the pipe."""
    import links   # imported here so the child opens its own Zenoh runtime
    import zenoh
    import os, sys, time
    debug = os.environ.get("RADIO_DEBUG") == "1"
    session = links.open_radio(subnet, routing_mode, lease_ms)
    if debug:
        print("[radio child] open_timeout", session.config().get_json("transport/unicast/open_timeout")
              if hasattr(session, "config") else "?", file=sys.stderr, flush=True)
    stats = {"put_max": 0.0, "last_rx": time.monotonic(), "rx_gap": 0.0, "t": time.monotonic()}
    publishers, subscribers = {}, {}
    send_lock = threading.Lock()

    def forward(sub_id, sample):
        if debug and str(sample.key_expr) not in publishers:   # remote traffic only
            now = time.monotonic()
            stats["rx_gap"] = max(stats["rx_gap"], now - stats["last_rx"])
            stats["last_rx"] = now
        try:
            with send_lock:
                conn.send(("msg", sub_id, str(sample.key_expr), bytes(sample.payload)))
        except (BrokenPipeError, EOFError, OSError):
            pass

    def subscribe(sub_id, key):
        # Channel handler + our own pulling thread, not a Python callback: with callbacks, Zenoh
        # threads call into Python while this thread declares the next subscriber, and at ~50
        # drones that occasionally deadlocked a drone's radio at startup (stuck in
        # declare_subscriber: the drone never joined).  A ring channel drops the oldest message
        # if we fall behind instead of blocking Zenoh (the radio is lossy anyway).
        sub = session.declare_subscriber(key, zenoh.handlers.RingChannel(1024))

        def pull():
            for sample in sub:          # ends when the subscriber is undeclared
                forward(sub_id, sample)
        threading.Thread(target=pull, name=f"radio-sub-{sub_id}", daemon=True).start()
        return sub

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
            subscribers[sub_id] = subscribe(sub_id, key)
        elif op == "unsub":
            sub = subscribers.pop(cmd[1], None)
            if sub is not None:
                sub.undeclare()
        elif op == "ping":
            try:
                with send_lock:
                    conn.send(("pong", 0, "", b""))
            except (BrokenPipeError, EOFError, OSError):
                break
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
    def __init__(self, radio, sub_id):
        self._radio, self._id = radio, sub_id

    def undeclare(self):
        self._radio._callbacks[self._id] = None
        self._radio._subs.pop(self._id, None)
        self._radio._send(("unsub", self._id))


class RadioProcess:
    SEND_QUEUE = 1000
    PING_S = 1.0          # the watchdog pings the radio process this often
    STALL_S = 5.0         # no answer for this long: the radio process is stuck, restart it
    START_GRACE_S = 10.0  # allowance for opening the Zenoh session

    def __init__(self, subnet=None, routing_mode=None, lease_ms=None):
        self._ctx = mp.get_context("spawn")   # never fork a process that already runs a Zenoh runtime
        self._args = (subnet, routing_mode, lease_ms)
        self._callbacks = []            # indexed by subscription id
        self._subs = {}                 # active subscription id -> key (re-declared after a restart)
        self._out = queue.Queue(maxsize=self.SEND_QUEUE)
        self._send_lock = threading.Lock()
        self._dropped = 0
        self.restarts = 0
        self._running = True
        self._start_child()
        threading.Thread(target=self._sender, name="radio-tx", daemon=True).start()
        threading.Thread(target=self._watchdog, name="radio-watchdog", daemon=True).start()

    # --- Session-like API ------------------------------------------------
    def declare_publisher(self, key):
        return _Publisher(self, key)

    def declare_subscriber(self, key, callback):
        self._callbacks.append(callback)
        sub_id = len(self._callbacks) - 1
        self._subs[sub_id] = key
        self._send(("sub", sub_id, key))
        return _Subscriber(self, sub_id)

    def put(self, key, data):
        self._send(("put", key, data))

    def close(self):
        self._running = False
        try:
            with self._send_lock:
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
    def _start_child(self):
        conn, child = self._ctx.Pipe()
        proc = self._ctx.Process(target=_worker, args=(child,) + self._args, name="radio", daemon=True)
        proc.start()
        child.close()
        with self._send_lock:
            self._conn, self._proc = conn, proc
            self._last_pong = time.monotonic() + self.START_GRACE_S
            for sub_id, key in list(self._subs.items()):    # a restarted session needs them again
                conn.send(("sub", sub_id, key))
        threading.Thread(target=self._receiver, args=(conn,), name="radio-rx", daemon=True).start()

    def _watchdog(self):
        """Restart the radio process if it stops answering.  At ~50 peers starting at once, a
        drone's Zenoh session occasionally hung inside a call (declare_subscriber, put) and the
        drone never joined; a fresh session joins normally."""
        while self._running:
            time.sleep(self.PING_S)
            if not self._running:
                return
            self._send(("ping",))
            silent = time.monotonic() - self._last_pong
            if silent > self.STALL_S or not self._proc.is_alive():
                self.restarts += 1
                log.warning("radio process %s for %.0fs; restarting it (restart %d)",
                            "gone" if not self._proc.is_alive() else "unresponsive", max(silent, 0), self.restarts)
                self._proc.kill()               # unblocks a sender stuck writing to its pipe
                self._proc.join(timeout=3)
                self._start_child()

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
                # One writer at a time on the pipe.  If the child is stuck this blocks with the lock
                # held; the watchdog kills the child first, which breaks the pipe and frees it.
                with self._send_lock:
                    self._conn.send(cmd)
            except (BrokenPipeError, OSError):
                time.sleep(0.1)                 # the watchdog is replacing the process (subs re-declared)

    def _receiver(self, conn):
        while self._running:
            try:
                op, sub_id, key, payload = conn.recv()
            except (EOFError, OSError):
                return
            if op == "pong":
                self._last_pong = time.monotonic()
                continue
            callback = self._callbacks[sub_id] if op == "msg" else None
            if callback is None:
                continue
            try:
                callback(Sample(key, payload))
            except Exception as e:
                log.warning("radio callback error on %s: %s", key, e)
