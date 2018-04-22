// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include "homework5/display/layer/road_graph_layer_base.h"
#include "homework5/display/user_interface_data.h"
#include "homework5/map/map_lib.h"

namespace utils {
namespace display {

class RoadGraphLayer : public RoadGraphLayerBase {
 public:
  RoadGraphLayer(const std::string& name, const homework5::map::MapLib* map_lib);
  ~RoadGraphLayer() override = default;

  void Draw() const override;

 private:
  DISALLOW_COPY_MOVE_AND_ASSIGN(RoadGraphLayer);
};

}  // namespace display
}  // namespace utils
