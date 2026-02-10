# ApproximateTime Algorithm

The ApproximateTime algorithm finds, for each group of messages (one from each topic), the set that minimizes the temporal spread. It does this incrementally as messages arrive, without waiting for all possible future messages.

## Terminology

### Message Queue

- **Topic queue**: A per-topic FIFO buffer that holds incoming messages in timestamp order.
- **Front message**: The oldest (earliest-timestamped) message currently in a topic's queue.

### Per-Iteration Variables

Recomputed at each iteration of the main loop (step 2):

- **`start_time`** / **`start_index`**: The smallest timestamp among all front messages, and the topic index of that message.
- **`end_time`** / **`end_index`**: The largest timestamp among all front messages, and the topic index of that message.

### Algorithm State

Persist across iterations within a single search:

- **Candidate**: The best set of messages found so far (one per topic). Its time span is recorded as **`candidate_start`** (the smallest timestamp in the set) and **`candidate_end`** (the largest timestamp in the set).
- **Pivot** / **`pivot_time`**: The topic index equal to `end_index` at the time the candidate was first formed. `pivot_time` is that topic's front message timestamp at that time. Once chosen, the pivot remains fixed until the candidate is published or discarded.
- **Past buffer**: A per-topic holding area for front messages that have been removed from their queues during the current search. These are restored when a candidate is published (step 6).

### Parameters

- **`max_interval_duration`**: The maximum allowed time span (`end_time - start_time`) for forming a candidate. Sets with wider spans are rejected.

- **`queue_size`**: The maximum number of messages buffered per topic. When exceeded, the oldest message is dropped (see [Queue overflow handling](#queue-overflow-handling)).

- **`age_penalty`** (default: 0.1): Controls the trade-off between temporal tightness and message recency. The value is used as a multiplicative factor on the "age" (how far `end_time` has advanced beyond `candidate_end`).  Higher values prefer newer message sets even if slightly less tightly clustered.

- **`inter_message_lower_bounds`**: Per-topic lower bounds on the interval between consecutive messages. This helps the algorithm prove candidate optimality without waiting for the next message to actually arrive, reducing output latency.

## Algorithm

Steps 2–4 repeat in a loop. Each iteration removes one front message (either discarding it or moving it to the past buffer) and recomputes.

1. **Wait for all topics**: Processing begins only when every topic has at least one buffered message.

2. **Compute interval bounds**: Compute `start_time` / `start_index` and `end_time` / `end_index` from the front messages of all topics.

3. **Form a candidate** (when no candidate exists):
   - If `end_time - start_time > max_interval_duration`: the interval is too wide. Drop the front message at `start_index` permanently and return to step 1.
   - If the topic at `end_index` has previously dropped messages due to queue overflow: it is not suitable as a pivot, because the dropped message might have produced a tighter candidate. Drop the front message at `start_index` permanently and return to step 1.
   - Otherwise: record the current set of front messages as the candidate. Set the pivot to `end_index` and `pivot_time` to `end_time`. Move the front message at `start_index` to the past buffer and return to step 2.

4. **Try to improve and publish the candidate** (when a candidate already exists):

   **Improvement** — evaluate first:

   Evaluate whether the current set of front messages is better than the candidate:

   ```text
   (end_time - candidate_end) * (1 + age_penalty) >= (start_time - candidate_start)
   ```

   - If this inequality holds: the current set is **not better**. The increase in end time (age cost) outweighs the increase in start time (tightness gain). Keep the existing candidate.
   - If this inequality does not hold: the current set **is better**. Replace the candidate with the current set of front messages (the pivot remains unchanged).

   In either case, move the front message at `start_index` to the past buffer.

   **Publish checks** — evaluate after improvement:

   - **Pivot exhausted**: If `start_index == pivot`, all possible candidate sets for this pivot have been examined. Publish the candidate (go to step 6).
   - **Provably optimal**: If `(end_time - candidate_end) * (1 + age_penalty) >= (pivot_time - candidate_start)`, the age cost already exceeds the maximum possible tightness improvement. (The maximum improvement is `pivot_time - candidate_start` because `start_time` can advance at most to `pivot_time` before the pivot-exhausted condition triggers.) Publish the candidate (go to step 6).

   If neither publish condition is met, return to step 2.

5. **Handle empty queues** (when the loop at step 2 cannot proceed because some topics have no front message):

   If no candidate exists, wait for more messages. If a candidate exists, the algorithm uses `inter_message_lower_bounds` to estimate the earliest possible arrival time of the next message on each empty queue (floored at `pivot_time`). It then simulates advancement using these virtual timestamps, checking at each virtual step whether the provably-optimal condition (from step 4) holds. If optimality is proven, publish the candidate (go to step 6). If a virtual set that is better than the current candidate is found, optimality cannot be proven — wait for more messages to arrive.

6. **After publishing**: The candidate messages are delivered to the callback. All messages in the past buffer are returned to the front of their respective queues. The candidate messages (now at the front of each queue) are then removed, and the algorithm restarts from step 1.

### Queue overflow handling

When a topic's buffer exceeds `queue_size`, all messages in the past buffer are first returned to their respective queues. The oldest message on the overflowing topic is then dropped, and the current candidate (if any) is discarded. The algorithm restarts from step 1.
