# malloc(): unaligned tcache chunk detected - Bug Analysis & Fix

## Issue Description
When clicking "Hold Focus", the application crashes with the error:
```
malloc(): unaligned tcache chunk detected
Aborted (core dumped)
```

This is a **heap corruption error** that indicates memory management problems in your autofocus code.

## Root Causes Identified

### 1. **Uninitialized Global Buffers** (PRIMARY CAUSE)
Located in `src/autofocus.cpp` lines 75-79:
```cpp
unsigned char *img_buf = (unsigned char *)malloc(img_size);
unsigned char *img_get_buf = (unsigned char *)malloc(img_size);
unsigned char *img_calc_buf = (unsigned char *)malloc(img_size);
```
- These buffers are allocated globally but **NOT initialized to zero**
- When accessed before first write, they may contain garbage values
- This can cause heap metadata corruption if buffers are accessed out of bounds

### 2. **Thread Safety Issues** (SECONDARY CAUSE)
- `img_calc_buf` is accessed by the main autofocus thread WITHOUT mutex protection
- The `computeBestFocusEigenLM()` function receives a reference to this buffer
- If the buffer is freed/reallocated while being processed, it causes heap corruption

### 3. **Lack of Buffer Protection During Processing**
In the `run()` function:
```cpp
if (tiltedcam1.getLatestFrame(img_calc_buf, img_size)) {
    cv::Mat image(imHeight, imWidth, CV_8UC1, img_calc_buf);
    // ... buffer can be modified here by another thread
    double locBestFocusDouble = computeBestFocusEigenLM(image, imHeight, imWidth);
}
```
- The buffer is passed to OpenCV Mat without copying
- OpenCV Mat just references the buffer without owning it
- If another thread modifies `img_calc_buf`, it corrupts the data being processed

## The Fix Applied

### 1. Initialize Global Buffers in Constructor
Added buffer initialization in the `autofocus::autofocus()` constructor:
```cpp
autofocus::autofocus()
    : lens1(), tiltedcam1(), stop_thread(false), settings("") {

  // Initialize global image buffers to zero to prevent heap corruption
  if (img_buf != nullptr && img_get_buf != nullptr && img_calc_buf != nullptr) {
    memset(img_buf, 0, img_size);
    memset(img_get_buf, 0, img_size);
    memset(img_calc_buf, 0, img_size);
  }
  // ... rest of constructor
}
```
This ensures buffers are zero-initialized before any use, preventing undefined behavior.

### 2. Add Mutex for Thread-Safe Access
Created a dedicated mutex for buffer protection:
```cpp
std::mutex img_calc_buf_mutex;  // Protect img_calc_buf access
```

### 3. Protect Critical Section
Protected the buffer access with mutex lock and cloning:
```cpp
{
  std::lock_guard<std::mutex> lock(img_calc_buf_mutex);
  
  // Create a copy of the buffer data to avoid race conditions
  cv::Mat image(imHeight, imWidth, CV_8UC1, img_calc_buf);
  cv::Mat imageCopy = image.clone();  // Clone to prevent buffer being freed while processing
  
  double locBestFocusDouble = computeBestFocusEigenLM(imageCopy, imHeight, imWidth);
  // ... rest of processing uses imageCopy, not original buffer
}
```

## Why This Fixes the Problem

1. **Buffer Initialization**: Prevents undefined behavior from garbage values
2. **Mutex Protection**: Ensures only one thread accesses the buffer at a time
3. **Buffer Cloning**: Creates an independent copy so the original buffer can be safely reused while processing
4. **RAII Lock**: `std::lock_guard` ensures mutex is always released even if exception occurs

## Testing the Fix

To verify the fix works:

1. Build the project:
   ```bash
   cd /home/hvi/repos/autofocus-merged/build
   cmake ..
   make -j$(nproc)
   ```

2. Run the application and:
   - Click on "Hold Focus"
   - The error should no longer occur
   - Focus should track smoothly

## Additional Recommendations

1. **Consider using `std::vector` instead of `malloc()`**:
   ```cpp
   std::vector<unsigned char> img_buf(img_size);
   ```
   This is RAII-compliant and safer.

2. **Add address sanitizer for debugging**:
   Compile with `-fsanitize=address` to catch memory errors early.

3. **Regular code review**: Have someone review multi-threaded memory access patterns.

## References

- [tcache corruption detection in glibc](https://sourceware.org/git/?p=glibc.git;a=commit;h=d6db68d6)
- [malloc thread safety](https://man7.org/linux/man-pages/man3/malloc.3.html)
- [RAII and mutex](https://en.cppreference.com/w/cpp/thread/lock_guard)
