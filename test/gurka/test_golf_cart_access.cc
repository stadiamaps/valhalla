#include "exceptions.h"
#include "gurka.h"
#include "sif/lowspeedvehiclecost.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

// Check that golf carts have access to the correct roads
class GolfCartAccess : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 500;

    // A network with varying levels of golf cart access
    const std::string ascii_map = R"(
        A--B--C--D--E--F--G--H--I--J
                          |  |  |  |
                          K--L--M--N
                                |
                                O---P
                                |   |
                                Q---R
                                |\
                                S-T
                                | |
                                U-V
    )";
    const gurka::ways ways = {{"AB",
                               {{"highway", "residential"}
                               }},
                              {"BC",
                               {{"highway", "living_street"},
                               }},
                              {"CD",
                               {{"highway", "tertiary"},
                               }},
                              {"DE",
                               {{"highway", "service"},
                               }},
                              {"EF",
                               {{"highway", "service"},
                                   {"service", "parking_aisle"},  // Should be allowed
                               }},
                              {"FG",
                               {{"highway", "path"},
                               }},
                              {"GH",
                               {{"highway", "path"},
                                   {"surface", "dirt"},  // not allowed
                               }},
                              {"HI",
                               {{"highway", "secondary"},
                               }},
                              {"IJ",
                               {{"highway", "path"},
                                   {"surface", "paved"},
                                   {"motor_vehicle", "no"},  // Tricky; motor_vehicle=no...
                                   {"golf_cart", "yes"},     // ... but golf_cart=yes
                               }},

                              // N/S cross streets
                              {"GK",
                               {{"highway", "residential"},
                               }},
                              {"HL",
                               {{"highway", "residential"},
                               }},
                              {"IM",
                               {{"highway", "primary"},
                               }},
                              {"JN",
                               {{"highway", "residential"},
                               }},

                              // Second set of N/S connectors
                              {"MO",
                               {{"highway", "residential"},
                                 {"maxspeed", "30mph"}
                               }},
                              {"OQ",
                               {{"highway", "residential"},
                                   {"maxspeed", "40mph"}  // Exceeds default max legal speed of road for golf cart use
                               }},
                              {"PR",
                               {{"highway", "residential"},
                               }},

                              // Middle E/W highway segments
                              {"KL",
                               {{"highway", "residential"},
                               }},
                              {"LM",
                               {{"highway", "primary"},  // No access for golf carts
                               }},
                              {"MN",
                               {{"highway", "residential"},
                               }},

                              // Lower
                              {"OP",
                               {{"highway", "residential"},
                               }},
                              {"QR",
                               {{"highway", "residential"},
                               }},

                              // Triangle
                              {"QS",
                               {{"highway", "residential"},
                                   {"golf_cart", "private"},
                               }},
                              {"QT",
                               {{"highway", "secondary"},
                                   {"maxspeed", "30mph"}
                               }},
                              {"ST",
                               {{"highway", "residential"},
                               }},
                              {"SU",
                               {{"highway", "residential"},
                                   {"access", "private"},
                               }},
                              {"TV",
                               {{"highway", "path"},
                                   {"golf", "cartpath"},
                               }},
                              {"UV",
                               {{"highway", "secondary"},
                               }}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/lsv_access");
  }
};

gurka::map GolfCartAccess::map = {};

/*************************************************************/

TEST_F(GolfCartAccess, CheckGolfCartAccess) {
  std::unordered_map<std::string, std::string> options = {
      {"/costing_options/low_speed_vehicle/vehicle_type", "golf_cart"}};
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "U"}, "low_speed_vehicle", options);
  gurka::assert::osrm::expect_steps(result, {"AB", "GK", "KL", "HL", "HI", "JN", "MN", "MO", "OP", "PR", "QR", "QT", "TV", "UV"});
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EF", "FG", "GK", "KL", "HL", "HI", "IJ", "JN", "MN", "MO", "OP", "PR", "QR", "QT", "TV", "UV"});
}

TEST_F(GolfCartAccess, CheckGolfCartAccessReducedSpeedLimit) {
  try {
    std::unordered_map<std::string, std::string> options = {
      {"/costing_options/low_speed_vehicle/max_allowed_speed_limit", "47"},
      {"/costing_options/low_speed_vehicle/vehicle_type", "golf_cart"}};
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "U"}, "low_speed_vehicle", options);
    gurka::assert::raw::expect_path(result, {"Unexpected path found"});
  } catch (const std::runtime_error& e) {
    EXPECT_STREQ(e.what(), "No path could be found for input");
  }
}

TEST(Standalone, HighwayCrossing) {
  constexpr double gridsize = 500;

  const std::string ascii_map = R"(
                F
                |
        A---B---C---D---E
                |
                G
    )";

  const gurka::ways ways = {
      // Tertiary highway
      {"AB", {{"highway", "tertiary"}}}, {"BC", {{"highway", "tertiary"}}},
      {"CD", {{"highway", "tertiary"}}}, {"DE", {{"highway", "tertiary"}}},
      // Motorway (not allowed for golf carts)
      {"FC", {{"highway", "motorway"}}}, {"CG", {{"highway", "motorway"}}},

  };

  const gurka::nodes nodes = {{"C",
                               {
                                   {"highway", "crossing"},
                               }}};

  const auto layout =
      gurka::detail::map_to_coordinates(ascii_map, gridsize, {5.1079374, 52.0887174});
  auto map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/lsv_access_highway_crossing");

  std::unordered_map<std::string, std::string> options = {
      {"/costing_options/low_speed_vehicle/vehicle_type", "golf_cart"}};
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "low_speed_vehicle", options);
  gurka::assert::osrm::expect_steps(result, {"AB"});
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE"});
}

TEST(Standalone, ConditionalGolfCartDestinationOnly) {
  const auto layout = gurka::detail::map_to_coordinates(R"(
    A---B-2-C---D
        |   |
        E---F
  )", 100, {5.1079374, 52.0887174});
  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"B2C", {{"highway", "residential"},
              {"golf_cart:conditional", "destination @ (Mo-Su 09:00-17:00)"}}},
      {"CD", {{"highway", "residential"}}},
      {"BE", {{"highway", "residential"}}},
      {"EF", {{"highway", "residential"}}},
      {"FC", {{"highway", "residential"}}},
  };
  const auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/lsv_conditional_destination",
                        {{"mjolnir.timezone", VALHALLA_BUILD_DIR "test/data/tz.sqlite"}});
  for (const auto* date_time_type : {"1", "2", "3"}) {
    SCOPED_TRACE(date_time_type);
    std::unordered_map<std::string, std::string> options{
        {"/costing_options/low_speed_vehicle/vehicle_type", "golf_cart"},
        {"/date_time/type", date_time_type},
        {"/date_time/value", "2020-04-02T12:00"},
    };
    auto route = gurka::do_action(Options::route, map, {"A", "D"}, "low_speed_vehicle", options);
    gurka::assert::raw::expect_path(route, {"AB", "BE", "EF", "FC", "CD"});

    route = gurka::do_action(Options::route, map, {"A", "2"}, "low_speed_vehicle", options);
    gurka::assert::raw::expect_path(route, {"AB", "B2C"});

    options["/date_time/value"] = "2020-04-02T20:00";
    route = gurka::do_action(Options::route, map, {"A", "D"}, "low_speed_vehicle", options);
    gurka::assert::raw::expect_path(route, {"AB", "B2C", "CD"});
  }
}

TEST(Standalone, GolfCartRejectsUnsuitableSurfaceAtWaypoints) {
  const auto layout = gurka::detail::map_to_coordinates("A---1---2---B", 100);
  const gurka::ways ways = {{"A12B", {{"highway", "path"}, {"surface", "dirt"}}}};
  const auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/lsv_unsuitable_surface");
  EXPECT_THROW(gurka::do_action(Options::route, map, {"1", "2"}, "low_speed_vehicle",
                                {{"/costing_options/low_speed_vehicle/vehicle_type", "golf_cart"}}),
               valhalla_exception_t);
}

TEST(Standalone, GolfCartParkingAisleTransitionCosts) {
  Costing options;
  options.mutable_options()->set_destination_only_penalty(600);
  options.mutable_options()->set_gate_cost(30);
  const auto costing = sif::CreateLowSpeedVehicleCost(options);
  const sif::EdgeLabel label;
  const baldr::DirectedEdge approach{};
  baldr::DirectedEdge aisle{};
  aisle.set_use(baldr::Use::kParkingAisle);
  aisle.set_dest_only(true);
  baldr::NodeInfo node{};
  ASSERT_EQ(aisle.stopimpact(0), 0);

  const auto forward = costing->TransitionCost(&aisle, &node, label, {}, {}, true);
  const auto reverse = costing->TransitionCostReverse(0, &node, &approach, &aisle, label, {}, {},
                                                       false, sif::InternalTurn::kNoTurn, false);
  EXPECT_FLOAT_EQ(forward.cost, 30.0f);
  EXPECT_FLOAT_EQ(reverse.cost, 30.0f);

  node.set_type(baldr::NodeType::kGate);
  const auto gated_forward = costing->TransitionCost(&aisle, &node, label, {}, {}, true);
  const auto gated_reverse = costing->TransitionCostReverse(0, &node, &approach, &aisle, label, {},
                                                             {}, false, sif::InternalTurn::kNoTurn,
                                                             false);
  EXPECT_FLOAT_EQ(gated_forward.secs - forward.secs, 30.0f);
  EXPECT_FLOAT_EQ(gated_reverse.secs - reverse.secs, 30.0f);
}
