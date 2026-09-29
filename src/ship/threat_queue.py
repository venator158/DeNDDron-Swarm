"""Min-heap of tracked threats keyed by time of closest point of approach.

All tracks share one clock, so ordering by absolute t_cpa is the same as
ordering by TCPA (t_cpa - now) and never needs re-keying.  Removal is lazy:
removed ids are skipped when they reach the top.
"""

import heapq
import itertools
from typing import Iterator, List, Optional, Tuple


class ThreatQueue:
    def __init__(self):
        self._heap: List[Tuple[float, int, str]] = []
        self._live = {}                     # threat_id -> t_cpa
        self._seq = itertools.count()       # tie-break: detection order

    def push(self, threat_id: str, t_cpa: float) -> None:
        if threat_id in self._live:
            raise KeyError(f"{threat_id} already queued")
        self._live[threat_id] = t_cpa
        heapq.heappush(self._heap, (t_cpa, next(self._seq), threat_id))

    def remove(self, threat_id: str) -> None:
        self._live.pop(threat_id, None)

    def _prune(self) -> None:
        while self._heap and self._heap[0][2] not in self._live:
            heapq.heappop(self._heap)

    def peek(self) -> Optional[str]:
        """The most urgent live threat (smallest t_cpa)."""
        self._prune()
        return self._heap[0][2] if self._heap else None

    def pop(self) -> Optional[str]:
        self._prune()
        if not self._heap:
            return None
        _, _, threat_id = heapq.heappop(self._heap)
        del self._live[threat_id]
        return threat_id

    def ordered(self) -> List[str]:
        """Live threats, most urgent first (for display; does not modify the heap)."""
        return [tid for _, _, tid in heapq.nsmallest(len(self._heap), self._heap) if tid in self._live]

    def __contains__(self, threat_id: str) -> bool:
        return threat_id in self._live

    def __len__(self) -> int:
        return len(self._live)

    def __iter__(self) -> Iterator[str]:
        return iter(self.ordered())
