// The one translation unit that compiles stb_image / stb_image_write (PNG/JPEG I/O, MJPEG frames from the Pi).
#pragma warning(push, 0)
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image.h"
#include "stb/stb_image_write.h"
#pragma warning(pop)
