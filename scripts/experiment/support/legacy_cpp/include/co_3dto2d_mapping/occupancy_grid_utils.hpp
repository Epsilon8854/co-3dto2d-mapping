#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <utility>
#include <vector>

namespace co_3dto2d_mapping::occupancy
{

struct Cell
{
  int col;
  int row;

  bool operator==(const Cell &other) const
  {
    return col == other.col && row == other.row;
  }
};

inline std::vector<Cell> traceGridLine(Cell start, Cell end, int width, int height)
{
  std::vector<Cell> cells;
  if (width <= 0 || height <= 0) {
    return cells;
  }

  int col = start.col;
  int row = start.row;
  const int delta_col = std::abs(end.col - start.col);
  const int step_col = start.col < end.col ? 1 : -1;
  const int delta_row = -std::abs(end.row - start.row);
  const int step_row = start.row < end.row ? 1 : -1;
  int error = delta_col + delta_row;

  while (true) {
    if (col >= 0 && row >= 0 && col < width && row < height) {
      cells.push_back({col, row});
    }
    if (col == end.col && row == end.row) {
      break;
    }

    const int doubled_error = 2 * error;
    if (doubled_error >= delta_row) {
      error += delta_row;
      col += step_col;
    }
    if (doubled_error <= delta_col) {
      error += delta_col;
      row += step_row;
    }
  }

  return cells;
}

inline std::pair<double, double> clipRayEndpoint(
    double origin_x,
    double origin_y,
    double hit_x,
    double hit_y,
    double max_range_m)
{
  const double delta_x = hit_x - origin_x;
  const double delta_y = hit_y - origin_y;
  const double range_m = std::hypot(delta_x, delta_y);
  if (max_range_m <= 0.0 || range_m <= max_range_m || range_m == 0.0) {
    return {hit_x, hit_y};
  }

  const double scale = max_range_m / range_m;
  return {origin_x + scale * delta_x, origin_y + scale * delta_y};
}

inline void markRayFree(
    std::vector<int8_t> &data,
    int width,
    const std::vector<Cell> &cells,
    const std::optional<Cell> &occupied_endpoint,
    int8_t free_value,
    int8_t occupied_value,
    bool clear_occupied)
{
  for (const Cell &cell : cells) {
    if (occupied_endpoint && cell == *occupied_endpoint) {
      continue;
    }

    const size_t index = static_cast<size_t>(cell.row) * static_cast<size_t>(width) +
        static_cast<size_t>(cell.col);
    if (!clear_occupied && data[index] == occupied_value) {
      continue;
    }
    data[index] = free_value;
  }
}

}  // namespace co_3dto2d_mapping::occupancy
