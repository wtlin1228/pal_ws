#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace cell_perception {

struct Params {
  double cell = 0.01, deck_x = 0.8, deck_y = 0.6, margin = 0.1;   // 1 cm grid over the deck plus a margin
  double layer_tol = 0.03, min_area = 0.015, min_height = 0.03;
};

struct Detection { double x, y, z_top, h, sx, sy, yaw, fill; };

/// Fill enclosed holes of a 0/255 mask: flood the outside, whatever is left unflooded is a hole.
inline void fillHoles(cv::Mat & mask) {
  cv::Mat inv = ~mask, padded;
  cv::copyMakeBorder(inv, padded, 1, 1, 1, 1, cv::BORDER_CONSTANT, 255);
  cv::floodFill(padded, cv::Point(0, 0), 0);
  mask |= padded(cv::Rect(1, 1, mask.cols, mask.rows));
}

/// Height map -> top-layer mask -> connected components -> rectangles. P: points in pallet_frame.
inline std::vector<Detection> detect(const std::vector<Eigen::Vector3d> & P, const Params & p = {}, cv::Mat * height_map = nullptr) {
  const int nx = std::lround((p.deck_x + 2 * p.margin) / p.cell), ny = std::lround((p.deck_y + 2 * p.margin) / p.cell);
  cv::Mat hm(ny, nx, CV_32F, cv::Scalar(0.0f));                                   // highest point per cell, rows = y
  cv::Mat lo(ny, nx, CV_32F, cv::Scalar(std::numeric_limits<float>::infinity()));  // lowest point per cell
  for (const auto & q : P) {
    const int ix = std::floor((q.x() + p.deck_x / 2 + p.margin) / p.cell);
    const int iy = std::floor((q.y() + p.deck_y / 2 + p.margin) / p.cell);
    if (ix < 0 || ix >= nx || iy < 0 || iy >= ny) continue;
    float & h = hm.at<float>(iy, ix); float & l = lo.at<float>(iy, ix);
    h = std::max(h, static_cast<float>(q.z())); l = std::min(l, static_cast<float>(q.z()));
  }
  if (height_map) *height_map = hm;
  double z_top; cv::minMaxLoc(hm, nullptr, &z_top);
  std::vector<Detection> out;
  if (z_top < p.min_height) return out;                                           // empty pallet

  const float top = static_cast<float>(z_top - p.layer_tol);
  cv::Mat mask = (hm > top) & (lo > top);           // a top cell has ALL its points at the top: edge and gap cells drop out
  fillHoles(mask);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::Mat::ones(3, 3, CV_8U));       // cut 1-cell bridges, drop specks
  cv::Mat labels;
  const int n = cv::connectedComponents(mask, labels, 4);

  for (int k = 1; k < n; ++k) {
    cv::Mat core = (labels == k);
    if (cv::countNonZero(core) * p.cell * p.cell < p.min_area) continue;
    cv::Mat blob; cv::dilate(core, blob, cv::Mat::ones(3, 3, CV_8U));              // give back the stripped edge cells
    std::vector<cv::Point> pts; cv::findNonZero(blob, pts);
    double yaw = std::remainder(cv::minAreaRect(pts).angle * M_PI / 180.0, M_PI / 2);   // orientation only, folded to (-45, 45]
    if (std::abs(yaw) < 3.0 * M_PI / 180.0) yaw = 0.0;                                // axis-aligned: a stray cell must not tilt it
    // size: project the cells onto the box axes and trim the 2 % outliers on each end, so one noisy row cannot stretch it
    std::vector<double> u, v;
    const double cs = std::cos(yaw), sn = std::sin(yaw);
    for (const auto & q : pts) { u.push_back(q.x * cs + q.y * sn); v.push_back(-q.x * sn + q.y * cs); }
    auto span = [](std::vector<double> & a) { std::sort(a.begin(), a.end()); const std::size_t k = a.size() / 50; return a[a.size() - 1 - k] - a[k]; };
    double sx = (span(u) + 1) * p.cell, sy = (span(v) + 1) * p.cell;              // +1: spans are between cell centers
    if (sx < sy) { std::swap(sx, sy); yaw += M_PI / 2; }                           // canonical: x is the long side
    yaw = std::remainder(yaw, M_PI);                                                // (-90 deg, 90 deg]
    const cv::Point2f center = cv::minAreaRect(pts).center;

    std::vector<cv::Point> core_pts; cv::findNonZero(core, core_pts);
    std::vector<float> z; for (const auto & c : core_pts) z.push_back(hm.at<float>(c));
    std::nth_element(z.begin(), z.begin() + z.size() / 2, z.end());
    const double z_top_k = z[z.size() / 2];

    cv::Mat ring; cv::dilate(blob, ring, cv::Mat::ones(7, 7, CV_8U)); ring &= ~blob;   // a 3 cm band around the box
    double z_bottom = 0.0;                                                          // nearest lower surface, else the deck
    for (int y = 0; y < ny; ++y) for (int x = 0; x < nx; ++x)
      if (ring.at<uint8_t>(y, x) && hm.at<float>(y, x) < z_top_k - p.layer_tol) z_bottom = std::max(z_bottom, double(hm.at<float>(y, x)));

    out.push_back({(center.x + 0.5) * p.cell - p.deck_x / 2 - p.margin, (center.y + 0.5) * p.cell - p.deck_y / 2 - p.margin,
                   z_top_k, z_top_k - z_bottom, sx, sy, yaw, cv::countNonZero(blob) * p.cell * p.cell / (sx * sy)});
  }
  return out;
}

}  // namespace cell_perception
