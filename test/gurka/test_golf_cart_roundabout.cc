#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

// Test that golf cart roundabout exit counts are correct when exits
// have kGolfCartAccess but NOT kAutoAccess (e.g., multi-modal paths).
class GolfCartRoundabout : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 100;

    // Standard roundabout layout with 3 exits beyond the entry point.
    // All edges are golf-cart paths with no general motor vehicle access.
    const std::string ascii_map = R"(
            I
            |
            H
           / \
       A--B   E--F
           \ /
            C
            |
            D
    )";

    const gurka::ways ways = {
        {"AB",
         {{"highway", "path"},
          {"surface", "paved"},
          {"motor_vehicle", "no"},
          {"golf_cart", "yes"},
          {"name", "Cart Path West"}}},
        {"BCEHB",
         {{"highway", "path"},
          {"surface", "paved"},
          {"motor_vehicle", "no"},
          {"golf_cart", "yes"},
          {"oneway", "yes"},
          {"junction", "roundabout"},
          {"name", ""}}},
        {"CD",
         {{"highway", "path"},
          {"surface", "paved"},
          {"motor_vehicle", "no"},
          {"golf_cart", "yes"},
          {"name", "Cart Path South"}}},
        {"EF",
         {{"highway", "path"},
          {"surface", "paved"},
          {"motor_vehicle", "no"},
          {"golf_cart", "yes"},
          {"name", "Cart Path East"}}},
        {"HI",
         {{"highway", "path"},
          {"surface", "paved"},
          {"motor_vehicle", "no"},
          {"golf_cart", "yes"},
          {"name", "Cart Path North"}}},
    };

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_golf_cart_roundabout",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map GolfCartRoundabout::map = {};

/*************************************************************/

// Golf cart: A→D should be the 1st exit
TEST_F(GolfCartRoundabout, GolfCart1stExit) {
  std::unordered_map<std::string, std::string> options = {
      {"/costing_options/low_speed_vehicle/vehicle_type", "golf_cart"}};
  auto result =
      gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "low_speed_vehicle", options);
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRoundaboutEnter,
                                                DirectionsLeg_Maneuver_Type_kRoundaboutExit,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index,
      "Enter the roundabout and take the 1st exit onto Cart Path South.",
      "Enter the roundabout and take the 1st exit.",
      "Enter the roundabout and take the 1st exit onto Cart Path South.",
      "Enter the roundabout and take the 1st exit onto Cart Path South.", "");
}

// Golf cart: A→F should be the 2nd exit
TEST_F(GolfCartRoundabout, GolfCart2ndExit) {
  std::unordered_map<std::string, std::string> options = {
      {"/costing_options/low_speed_vehicle/vehicle_type", "golf_cart"}};
  auto result =
      gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "low_speed_vehicle", options);
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRoundaboutEnter,
                                                DirectionsLeg_Maneuver_Type_kRoundaboutExit,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index,
      "Enter the roundabout and take the 2nd exit onto Cart Path East.",
      "Enter the roundabout and take the 2nd exit.",
      "Enter the roundabout and take the 2nd exit onto Cart Path East.",
      "Enter the roundabout and take the 2nd exit onto Cart Path East.", "");
}

// Golf cart: A→I should be the 3rd exit
TEST_F(GolfCartRoundabout, GolfCart3rdExit) {
  std::unordered_map<std::string, std::string> options = {
      {"/costing_options/low_speed_vehicle/vehicle_type", "golf_cart"}};
  auto result =
      gurka::do_action(valhalla::Options::route, map, {"A", "I"}, "low_speed_vehicle", options);
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRoundaboutEnter,
                                                DirectionsLeg_Maneuver_Type_kRoundaboutExit,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index,
      "Enter the roundabout and take the 3rd exit onto Cart Path North.",
      "Enter the roundabout and take the 3rd exit.",
      "Enter the roundabout and take the 3rd exit onto Cart Path North.",
      "Enter the roundabout and take the 3rd exit onto Cart Path North.", "");
}

