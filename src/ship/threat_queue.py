"""Min-heap of tracked threats keyed by time of closest point of approach.

All tracks share one clock, so ordering by absolute t_cpa is the same as
ordering by TCPA (t_cpa - now) and never needs re-keying.  Removal is lazy:
removed or re-keyed entries are skipped when they reach the top.
"""

import heapq
import itertools
from typing import Iterator, List, Optional, Tuple


class ThreatQueue:
    def __init__(self):
        self._heap: List[Tuple[float, int, str]] = []
        self._live = {}                     # threat_id -> (t_cpa, seq) of its current heap entry
        self._seq = itertools.count()       # tie-break: detection order

    def push(self, threat_id: str, t_cpa: float) -> None:
        if threat_id in self._live:
            raise KeyError(f"{threat_id} already queued")
        entry = (t_cpa, next(self._seq), threat_id)
        self._live[threat_id] = entry[:2]
        heapq.heappush(self._heap, entry)

    def update(self, threat_id: str, t_cpa: float) -> None:
        """Re-key a live threat (its track changed); the old heap entry goes stale."""
        self.remove(threat_id)
        self.push(threat_id, t_cpa)

    def _is_live(self, entry) -> bool:
        return self._live.get(entry[2]) == entry[:2]

    def remove(self, threat_id: str) -> None:
        self._live.pop(threat_id, None)

    def _prune(self) -> None:
        while self._heap and not self._is_live(self._heap[0]):
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
        return [e[2] for e in heapq.nsmallest(len(self._heap), self._heap) if self._is_live(e)]

    def __contains__(self, threat_id: str) -> bool:
        return threat_id in self._live

    def __len__(self) -> int:
        return len(self._live)

    def __iter__(self) -> Iterator[str]:
        return iter(self.ordered())
