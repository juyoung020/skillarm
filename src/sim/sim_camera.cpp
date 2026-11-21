#include "sim/sim_camera.hpp"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <cmath>
#include <cstring>
#include <stdexcept>

namespace skillarm {
namespace {
constexpr double kPi = 3.14159265358979323846;

LRESULT CALLBACK wnd_proc(HWND h, UINT msg, WPARAM w, LPARAM l) { return DefWindowProcA(h, msg, w, l); }
}  // namespace

SimCamera::SimCamera(const mjModel* m, const std::string& camera, int width, int height)
    : name_(camera), w_(width), h_(height) {
  cam_id_ = mj_name2id(m, mjOBJ_CAMERA, camera.c_str());
  if (cam_id_ < 0) throw std::runtime_error("SimCamera: no camera '" + camera + "' in the model");

  // hidden window + legacy WGL context; MuJoCo renders into its own offscreen framebuffer
  static const char* kClass = "skillarm_gl";
  static bool registered = false;
  HINSTANCE inst = GetModuleHandleA(nullptr);
  if (!registered) {
    WNDCLASSA wc{};
    wc.style = CS_OWNDC;
    wc.lpfnWndProc = wnd_proc;
    wc.hInstance = inst;
    wc.lpszClassName = kClass;
    RegisterClassA(&wc);
    registered = true;
  }
  HWND hwnd = CreateWindowExA(0, kClass, "skillarm", WS_OVERLAPPEDWINDOW, 0, 0, 16, 16, nullptr, nullptr, inst, nullptr);
  if (!hwnd) throw std::runtime_error("SimCamera: CreateWindow failed");
  HDC hdc = GetDC(hwnd);
  PIXELFORMATDESCRIPTOR pfd{};
  pfd.nSize = sizeof pfd;
  pfd.nVersion = 1;
  pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
  pfd.iPixelType = PFD_TYPE_RGBA;
  pfd.cColorBits = 24;
  pfd.cDepthBits = 24;
  pfd.cStencilBits = 8;
  const int pf = ChoosePixelFormat(hdc, &pfd);
  if (!pf || !SetPixelFormat(hdc, pf, &pfd)) throw std::runtime_error("SimCamera: no OpenGL pixel format");
  HGLRC rc = wglCreateContext(hdc);
  if (!rc || !wglMakeCurrent(hdc, rc)) throw std::runtime_error("SimCamera: wglCreateContext failed");
  hwnd_ = hwnd;
  hdc_ = hdc;
  hglrc_ = rc;

  mjv_defaultOption(&opt_);
  mjv_defaultScene(&scn_);
  mjv_makeScene(m, &scn_, 10000);
  mjr_defaultContext(&con_);
  mjr_makeContext(m, &con_, mjFONTSCALE_100);
  mjr_resizeOffscreen(w_, h_, &con_);
  mjr_setBuffer(mjFB_OFFSCREEN, &con_);
  con_.readDepthMap = mjDEPTH_ZERONEAR;
  rgb_.resize(size_t(w_) * h_ * 3);
  depth_.resize(size_t(w_) * h_);
}

SimCamera::~SimCamera() {
  if (hglrc_) wglMakeCurrent(static_cast<HDC>(hdc_), static_cast<HGLRC>(hglrc_));
  mjr_freeContext(&con_);
  mjv_freeScene(&scn_);
  if (hglrc_) {
    wglMakeCurrent(nullptr, nullptr);
    wglDeleteContext(static_cast<HGLRC>(hglrc_));
  }
  if (hdc_) ReleaseDC(static_cast<HWND>(hwnd_), static_cast<HDC>(hdc_));
  if (hwnd_) DestroyWindow(static_cast<HWND>(hwnd_));
}

CameraObservation SimCamera::capture(const mjModel* m, mjData* d) {
  wglMakeCurrent(static_cast<HDC>(hdc_), static_cast<HGLRC>(hglrc_));
  mjvCamera cam;
  mjv_defaultCamera(&cam);
  cam.type = mjCAMERA_FIXED;
  cam.fixedcamid = cam_id_;
  mjv_updateScene(m, d, &opt_, nullptr, &cam, mjCAT_ALL, &scn_);
  const mjrRect vp = {0, 0, w_, h_};
  mjr_setBuffer(mjFB_OFFSCREEN, &con_);
  mjr_render(vp, &scn_, &con_);
  mjr_readPixels(rgb_.data(), depth_.data(), vp, &con_);

  CameraObservation o;
  o.name = name_;
  o.timestamp_s = d->time;
  o.rgb.width = w_;
  o.rgb.height = h_;
  o.rgb.rgb.resize(rgb_.size());
  o.depth_m.resize(depth_.size());
  // OpenGL rows start at the bottom; window depth [0,1] -> metric z along the optical axis
  const double ext = m->stat.extent, zn = m->vis.map.znear * ext, zf = m->vis.map.zfar * ext;
  const size_t row = size_t(w_) * 3;
  for (int r = 0; r < h_; ++r) {
    std::memcpy(o.rgb.rgb.data() + size_t(r) * row, rgb_.data() + size_t(h_ - 1 - r) * row, row);
    const float* src = depth_.data() + size_t(h_ - 1 - r) * w_;
    float* dst = o.depth_m.data() + size_t(r) * w_;
    for (int c = 0; c < w_; ++c) {
      const double z = zn / (1.0 - src[c] * (1.0 - zn / zf));
      dst[c] = src[c] < 1.0f ? float(z) : 0.f;  // far plane = nothing hit
    }
  }

  const double fy = 0.5 * h_ / std::tan(0.5 * m->cam_fovy[cam_id_] * kPi / 180.0);
  o.intrinsics = {fy, 0, 0.5 * w_ - 0.5, 0, fy, 0.5 * h_ - 0.5, 0, 0, 1};

  // MuJoCo camera: x right, y up, looks along -z. OpenCV: y down, z forward -> negate columns y and z.
  const mjtNum* R = d->cam_xmat + 9 * cam_id_;
  mjtNum Rcv[9];
  for (int i = 0; i < 3; ++i) {
    Rcv[3 * i + 0] = R[3 * i + 0];
    Rcv[3 * i + 1] = -R[3 * i + 1];
    Rcv[3 * i + 2] = -R[3 * i + 2];
  }
  mjtNum q[4];
  mju_mat2Quat(q, Rcv);
  o.camera_pose.position = {d->cam_xpos[3 * cam_id_], d->cam_xpos[3 * cam_id_ + 1], d->cam_xpos[3 * cam_id_ + 2]};
  o.camera_pose.quaternion_wxyz = {q[0], q[1], q[2], q[3]};
  return o;
}

}  // namespace skillarm
