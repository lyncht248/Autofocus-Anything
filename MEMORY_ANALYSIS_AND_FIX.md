# Memory Leak Analysis - Test Results

## Critical Discovery: Memory NOT Being Released After Recording

### Key Observations from Logs

#### Recording 1: 1200 frames
```
START: Memory before recording: 193.15 MB
After RECORD_COMPLETE: Memory: 3432.58 MB
Memory used: ~3239 MB for 1200 frames (~2.7 MB/frame)
```

#### After Recording 1 - Switching to Live View
```
[Recorder::clearFrames()] clearing frames, size before clearing: 1200
[Recorder::clearFrames()] frames cleared, size after clearing: 0
[Recorder::clearFrames()] frame_times cleared, size after clearing: 0
✓ Frames ARE being cleared properly
```

#### BUT CRITICAL: Memory NOT released back to system!
```
[System::whenLiveViewToggled] Resetting frame processor
→ Memory STILL at ~2687.33 MB (not released!)
```

#### Recording 2 Baseline
```
START: Memory before recording: 2687.33 MB (previous baseline!)
After RECORD_COMPLETE: Memory: 4185.81 MB
Memory used: ~1498 MB for 1200 frames
BUT TOTAL IS NOW: 4185.81 MB (leaked ~753 MB from previous cycle)
```

#### Pattern Across All 5 Recording Cycles
```
Recording 1: 193 MB → 3432 MB (used 3239 MB) ✓ Expected
Recording 2: 2687 MB → 4185 MB (used 1498 MB) ✗ Baseline didn't return to ~193 MB!
Recording 3: 2811 MB → 6069 MB (used 3258 MB) ✗ Baseline continued rising
Recording 4: 4735 MB → 7931 MB (used 3196 MB) ✗ Baseline at 4.7 GB
Recording 5: 2989 MB → 5745 MB (used 2756 MB) ✗ Baseline at 3 GB (OS swapped out some)
```

## The Problem

**The memory is being cleared from the frames vector, BUT the physical memory pages are NOT being returned to the OS.**

This is NOT a reference counting issue. It's a **memory deallocation/fragmentation issue**.

### Why This Happens

1. **Memory allocated during recording**: 3+ GB of frames allocated
2. **Frames cleared from vector**: `frames.clear()` called ✓
3. **Shared_ptr destructors called**: References released ✓
4. **BUT: OS doesn't reclaim the pages**
   - Deleted memory stays in process heap
   - Heap fragmentation keeps memory mapped
   - Virtual address space stays allocated even if unused
   - `libc malloc` doesn't return memory to OS easily

## Root Cause

**The issue is NOT with the code - it's with memory fragmentation and heap management.**

When you allocate/deallocate large amounts of memory (3GB+), the heap becomes fragmented:
- 1200 frames × 2.7 MB = 3.24 GB allocated
- Each frame is separate `shared_ptr`/`CVD::Image` allocation
- After clearing, heap has many "holes"
- Heap fragmentation prevents returning memory to OS
- Therefore RSS stays high

## Evidence

### frame_times and current ARE being cleared:
```
[Recorder::clearFrames()] frame_times size: 1200
[Recorder::clearFrames()] frame_times cleared, size after clearing: 0
```

### Queues ARE empty:
```
[FrameProcessor::clearQueues] frameQueue size after clearing: 0
[FrameProcessor::clearQueues] stabQueue size after clearing: 0
[FrameProcessor::clearQueues] released queue size after clearing: 0
```

### Frame count IS correct:
```
[Recorder::clearFrames()] clearing frames, size before clearing: 1200
[Recorder::clearFrames()] frames cleared, size after clearing: 0
```

**But RSS doesn't decrease** → Memory fragmentation/heap issue, not a reference leak

## Solutions to Investigate

### Solution 1: Force Memory Deallocation (Likely to work)
Use `malloc_trim()` to force libc to return memory to OS:
```cpp
#include <malloc.h>

// After clearFrames():
malloc_trim(0);  // Force heap compaction and return to OS
```

### Solution 1 Works!

The memory leak is **NOT a reference counting issue** - it's a **heap fragmentation problem**.

### What Was Happening
1. Recording allocates 3+ GB of frame memory (1200 frames × ~2.7 MB each)
2. `Recorder::clearFrames()` clears the vector and releases all pointers
3. Shared_ptr destructors call `delete`
4. BUT: Linux libc doesn't return fragmented memory to OS
5. RSS stays high across multiple recordings
6. After 5 recordings: 7.9 GB used (was meant to be ~3 GB per recording)

## The Solution: malloc_trim()

Added `malloc_trim(0)` call after clearing frames in `Recorder::clearFrames()`:

```cpp
void Recorder::clearFrames() {
  frames.clear();
  frame_times.clear();
  current.reset();
  
  malloc_trim(0);  // ← NEW: Force heap compaction and return memory to OS
}
```

### How malloc_trim() Works
- Compacts the heap by consolidating free chunks
- Returns unused memory pages to the OS
- Reduces fragmentation
- Essential after allocating/deallocating large blocks
