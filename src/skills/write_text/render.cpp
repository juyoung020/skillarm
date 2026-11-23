#include "render.hpp"

#include <EGL/egl.h>
#include <EGL/eglext.h>

#include <cstring>
#include <vector>

namespace wt {
namespace {

EGLDisplay egl_display() {
  static EGLDisplay dpy = [] {
    auto query = reinterpret_cast<PFNEGLQUERYDEVICESEXTPROC>(eglGetProcAddress("eglQueryDevicesEXT"));
    auto get_dpy = reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(eglGetProcAddress("eglGetPlatformDisplayEXT"));
    EGLDisplay d = EGL_NO_DISPLAY;
    if (query && get_dpy) {
      EGLDeviceEXT devs[8];
      EGLint n = 0;
      if (query(8, devs, &n))
        for (EGLint i = 0; i < n && d == EGL_NO_DISPLAY; ++i) {
          EGLDisplay c = get_dpy(EGL_PLATFORM_DEVICE_EXT, devs[i], nullptr);
          EGLint maj = 0, mn = 0;
          if (c != EGL_NO_DISPLAY && eglInitialize(c, &maj, &mn)) d = c;
        }
    }
    if (d != EGL_NO_DISPLAY && !eglBindAPI(EGL_OPENGL_API)) d = EGL_NO_DISPLAY;
    return d;
  }();
  return dpy;
}

class EglRenderer final : public Renderer {
 public:
  EglRenderer(const mjModel* m, int w, int h, EGLContext ctx) : w_(w), h_(h), ctx_(ctx) {
    mjv_defaultOption(&opt_);
    mjv_defaultScene(&scn_);
    mjv_makeScene(m, &scn_, 4000);
    mjr_defaultContext(&con_);
    mjr_makeContext(m, &con_, mjFONTSCALE_100);
    mjr_resizeOffscreen(w_, h_, &con_);
    mjr_setBuffer(mjFB_OFFSCREEN, &con_);
    rgb_.resize(size_t(w_) * size_t(h_) * 3);
  }
  ~EglRenderer() override {
    eglMakeCurrent(egl_display(), EGL_NO_SURFACE, EGL_NO_SURFACE, ctx_);
    mjr_freeContext(&con_);
    mjv_freeScene(&scn_);
    eglMakeCurrent(egl_display(), EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    eglDestroyContext(egl_display(), ctx_);
  }
  bool capture(const mjModel* m, mjData* d, const std::string& camera, Image& out) override {
    const int cid = mj_name2id(m, mjOBJ_CAMERA, camera.c_str());
    if (cid < 0) return false;
    eglMakeCurrent(egl_display(), EGL_NO_SURFACE, EGL_NO_SURFACE, ctx_);
    mjvCamera cam;
    mjv_defaultCamera(&cam);
    cam.type = mjCAMERA_FIXED;
    cam.fixedcamid = cid;
    mjv_updateScene(m, d, &opt_, nullptr, &cam, mjCAT_ALL, &scn_);
    mjrRect vp{0, 0, w_, h_};
    mjr_render(vp, &scn_, &con_);
    mjr_readPixels(rgb_.data(), nullptr, vp, &con_);
    out = Image(w_, h_);
    for (int y = 0; y < h_; ++y)  // OpenGL rows are bottom-up
      std::memcpy(out.data() + size_t(y) * size_t(w_) * 3, rgb_.data() + size_t(h_ - 1 - y) * size_t(w_) * 3, size_t(w_) * 3);
    return true;
  }

 private:
  int w_, h_;
  EGLContext ctx_;
  mjvOption opt_;
  mjvScene scn_;
  mjrContext con_;
  std::vector<unsigned char> rgb_;
};

}  // namespace

std::unique_ptr<Renderer> Renderer::make(const mjModel* m, int w, int h) {
  EGLDisplay dpy = egl_display();
  if (dpy == EGL_NO_DISPLAY) return nullptr;
  const EGLint cfg_attr[] = {EGL_SURFACE_TYPE, EGL_PBUFFER_BIT, EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8,
                             EGL_DEPTH_SIZE, 24, EGL_STENCIL_SIZE, 8, EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT, EGL_NONE};
  EGLConfig cfg;
  EGLint n = 0;
  if (!eglChooseConfig(dpy, cfg_attr, &cfg, 1, &n) || n < 1) return nullptr;
  EGLContext ctx = eglCreateContext(dpy, cfg, EGL_NO_CONTEXT, nullptr);
  if (ctx == EGL_NO_CONTEXT || !eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, ctx)) return nullptr;
  return std::make_unique<EglRenderer>(m, w, h, ctx);
}

}  // namespace wt
