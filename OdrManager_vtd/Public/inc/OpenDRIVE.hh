/*
 * Copyright 2007 VIRES Simulationstechnologie GmbH
 * ALL RIGHTS RESERVED
 *
 * This source code ("Source Code") was originally derived from a
 * code base owned by VIRES Simulationstechnologie GmbH ("VIRES")
 * 
 * LICENSE: VIRES grants the user ("Licensee") permission to reproduce,
 * distribute, and create derivative works from this Source Code,
 * provided that: (1) the user reproduces this entire notice within
 * both source and binary format redistributions and any accompanying
 * materials such as documentation in printed or electronic format;
 * (2) the Source Code is not to be used, or ported or modified for
 * use, except in conjunction with the VIRES OdrManager; and (3) the
 * names of VIRES Simulationstechnologie GmbH and VIRES may not be used
 * in any advertising or publicity relating to the Source Code without the
 * prior written permission of VIRES. No further license or permission
 * may be inferred or deemed or construed to exist with regard to the
 * Source Code or the code base of which it forms a part. All rights
 * not expressly granted are reserved.
 * 
 * This Source Code is provided to Licensee AS IS, without any
 * warranty of any kind, either express, implied, or statutory,
 * including, but not limited to, any warranty that the Source Code
 * will conform to specifications, any implied warranties of
 * merchantability, fitness for a particular purpose, and freedom
 * from infringement, and any warranty that the documentation will
 * conform to the program, or any warranty that the Source Code will
 * be error free.
 * 
 * IN NO EVENT WILL VIRES BE LIABLE FOR ANY DAMAGES, INCLUDING, BUT NOT
 * LIMITED TO DIRECT, INDIRECT, SPECIAL OR CONSEQUENTIAL DAMAGES,
 * ARISING OUT OF, RESULTING FROM, OR IN ANY WAY CONNECTED WITH THE
 * SOURCE CODE, WHETHER OR NOT BASED UPON WARRANTY, CONTRACT, TORT OR
 * OTHERWISE, WHETHER OR NOT INJURY WAS SUSTAINED BY PERSONS OR
 * PROPERTY OR OTHERWISE, AND WHETHER OR NOT LOSS WAS SUSTAINED FROM,
 * OR AROSE OUT OF USE OR RESULTS FROM USE OF, OR LACK OF ABILITY TO
 * USE, THE SOURCE CODE.
 * 
 * VIRES Simulationstechnologie GmbH, Oberaustrasse 34, 83026 Rosenheim
 * Germany, opendrive@vires.com, www.vires.com
 */
/* ===================================================
 *  file:       OpenDRIVE.hh
 * ---------------------------------------------------
 *  purpose:	definitions for OpenDRIVE
 * ---------------------------------------------------
 *  first edit:	10.11.2005 by M. Dupuis @ VIRES GmbH
 *  last mod.:  21.06.2007 by M. Knerr @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_HH
#define _OPENDRIVE_HH

namespace OpenDrive
{
    // OPCODES
    enum EnOpcode
    {
        ODR_OPCODE_NONE =  0,
        ODR_OPCODE_HEADER,
        ODR_OPCODE_ROAD_HEADER,
        ODR_OPCODE_ROAD_LINK,
        ODR_OPCODE_ROAD_TYPE,
        ODR_OPCODE_GEO_HEADER,
        ODR_OPCODE_GEO_LINE,
        ODR_OPCODE_GEO_SPIRAL,
        ODR_OPCODE_GEO_ARC,
        ODR_OPCODE_ELEVATION,
        ODR_OPCODE_LANE_SECTION, // 10
        ODR_OPCODE_LANE,
        ODR_OPCODE_LANE_LINK,
        ODR_OPCODE_LANE_WIDTH,
        ODR_OPCODE_LANE_MATERIAL,
        ODR_OPCODE_LANE_VISIBILITY,
        ODR_OPCODE_SIGNAL,
        ODR_OPCODE_LANE_VALIDITY,
        ODR_OPCODE_SIGNAL_DEPEND,
        ODR_OPCODE_CONTROLLER,
        ODR_OPCODE_CONTROL_ENTRY, // 20
        ODR_OPCODE_JUNCTION_HEADER,
        ODR_OPCODE_JUNCTION_LINK,
        ODR_OPCODE_JUNCTION_PRIORITY,
        ODR_OPCODE_OBJECT,
        ODR_OPCODE_USER_DATA,
        ODR_OPCODE_JUNCTION_LANE_LINK,
        ODR_OPCODE_CROSSFALL,
        ODR_OPCODE_JUNCTION_CONTROL,
        ODR_OPCODE_LANE_ROAD_MARK,
        ODR_OPCODE_PREDECESSOR, // 30
        ODR_OPCODE_SUCCESSOR,
        ODR_OPCODE_LANES,
        ODR_OPCODE_LANES_LEFT,
        ODR_OPCODE_LANES_CENTER,
        ODR_OPCODE_LANES_RIGHT,
        ODR_OPCODE_PLANVIEW,
        ODR_OPCODE_ELEV_PROFILE,
        ODR_OPCODE_LATERAL_PROFILE,
        ODR_OPCODE_OBJECTS,
        ODR_OPCODE_SIGNALS,  // 40
        ODR_OPCODE_OPENDRIVE,
        ODR_OPCODE_SUPERELEVATION,
        ODR_OPCODE_GEO_POLY,
        ODR_OPCODE_LANE_SPEED,
        ODR_OPCODE_LANE_ACCESS,
        ODR_OPCODE_LANE_HEIGHT,
        ODR_OPCODE_CORNER_INERTIAL,
        ODR_OPCODE_CORNER_ROAD,
        ODR_OPCODE_CORNER_RELATIVE,
        ODR_OPCODE_TUNNEL,  // 50
        ODR_OPCODE_BRIDGE,
        ODR_OPCODE_SIGNAL_REFERENCE,
        ODR_OPCODE_OBJECT_OUTLINE,
        ODR_OPCODE_SURFACE,
        ODR_OPCODE_SURFACE_CRG,
        ODR_OPCODE_LANE_OFFSET,
        ODR_OPCODE_GENERIC_NODE,
        ODR_OPCODE_CORNER_LOCAL,
        ODR_OPCODE_REPEAT,
        ODR_OPCODE_GEO_PARAM_POLY,  // 60
        ODR_OPCODE_LANE_BORDER,
        ODR_OPCODE_ROAD_SPEED,
        ODR_OPCODE_GEO_REFERENCE,
        ODR_OPCODE_LATERAL_SHAPE,
        ODR_OPCODE_JUNCTION_GROUP,
        ODR_OPCODE_JUNCTION_REFERENCE,
        ODR_OPCODE_ROAD_MARK_TYPE,
        ODR_OPCODE_ROAD_MARK_LINE,
        ODR_OPCODE_PARKING_SPACE,
        ODR_OPCODE_PARKING_SPACE_MARKING,
        ODR_OPCODE_NEIGHBOR,
        ODR_OPCODE_RAILROAD,
        ODR_OPCODE_RAILROAD_SWITCH,
        ODR_OPCODE_SWITCH,
        ODR_OPCODE_RAILROAD_MAIN_TRACK,
        ODR_OPCODE_RAILROAD_SIDE_TRACK,
        ODR_OPCODE_OBJECT_REFERENCE,
        ODR_OPCODE_OBJECT_MATERIAL,
        ODR_OPCODE_LANE_RULE,
        ODR_OPCODE_OFFSET,
        ODR_OPCODE_ROAD_MARK_SWAY,
        ODR_OPCODE_ROAD_MARK_EXPLICIT,
        ODR_OPCODE_ROAD_MARK_EXPLICIT_LINE,
        ODR_OPCODE_MARKINGS,
        ODR_OPCODE_MARKINGS_MARKING,
        ODR_OPCODE_CORNER_REFERENCE,
        ODR_OPCODE_BORDERS,
        ODR_OPCODE_BORDER,
        ODR_OPCODE_REFERENCE,
        ODR_OPCODE_POSITION_ROAD,
        ODR_OPCODE_POSITION_INERTIAL,
        ODR_OPCODE_JUNCTION_LINK_PREDECESSOR,
        ODR_OPCODE_JUNCTION_LINK_SUCCESSOR,
        ODR_OPCODE_DATA_QUALITY,
        ODR_OPCODE_RAW_DATA_DESC,
        ODR_OPCODE_ERROR_DESC,
        ODR_OPCODE_OUTLINE,
        ODR_OPCODE_TRAFFIC_OBJECT = 2000
    };

    // GENERAL TYPE
    enum EnType {
        ODR_TYPE_NONE = 0,
        ODR_TYPE_ROAD,
        ODR_TYPE_JUNCTION
    };

    // LINK TYPE
    enum EnLinkType {
        ODR_LINK_TYPE_PREDECESSOR = 1,
        ODR_LINK_TYPE_SUCCESSOR,
        ODR_LINK_TYPE_NEIGHBOR
    };

    // LINK POINT
    enum EnLinkPoint {
        ODR_LINK_POINT_START = 0,
        ODR_LINK_POINT_END
    };

    // ROAD TYPE
    enum EnRoadType {
        ODR_ROAD_TYPE_NONE = 0,
        ODR_ROAD_TYPE_RURAL,
        ODR_ROAD_TYPE_MOTORWAY,
        ODR_ROAD_TYPE_TOWN,
        ODR_ROAD_TYPE_LOW_SPEED,
        ODR_ROAD_TYPE_PEDESTRIAN,
        ODR_ROAD_TYPE_TOWN_EXPRESSWAY,
        ODR_ROAD_TYPE_TOWN_COLLECTOR,
        ODR_ROAD_TYPE_TOWN_ARTERIAL,
        ODR_ROAD_TYPE_TOWN_PRIVATE,
        ODR_ROAD_TYPE_TOWN_LOCAL,
        ODR_ROAD_TYPE_TOWN_PLAYSTREET,      
        ODR_ROAD_TYPE_BICYCLE
    };

    // GEOMETRY TYPE
    enum EnGeometryType {
        ODR_GEO_TYPE_LINE = 0,
        ODR_GEO_TYPE_SPIRAL,
        ODR_GEO_TYPE_ARC
    };

    // ROAD MARK
    enum EnRoadMarkType {
        ODR_ROAD_MARK_TYPE_NONE=  0,
        ODR_ROAD_MARK_TYPE_SOLID,
        ODR_ROAD_MARK_TYPE_BROKEN,
        ODR_ROAD_MARK_TYPE_SOLID_SOLID,
        ODR_ROAD_MARK_TYPE_SOLID_BROKEN,
        ODR_ROAD_MARK_TYPE_BROKEN_SOLID,
        ODR_ROAD_MARK_TYPE_BROKEN_BROKEN,
        ODR_ROAD_MARK_TYPE_CURB,
        ODR_ROAD_MARK_TYPE_BOTTS_DOTS,
        ODR_ROAD_MARK_TYPE_GRASS,
        ODR_ROAD_MARK_TYPE_CUSTOM,
        ODR_ROAD_MARK_TYPE_EDGE
    };
    
    enum EnRoadMarkWeight {
        ODR_ROAD_MARK_WEIGHT_NONE = 0,
        ODR_ROAD_MARK_WEIGHT_STANDARD,
        ODR_ROAD_MARK_WEIGHT_BOLD
    };
    
    enum EnRoadMarkColor {
        ODR_ROAD_MARK_COLOR_NONE = 0,
        ODR_ROAD_MARK_COLOR_STANDARD,
        ODR_ROAD_MARK_COLOR_YELLOW,
        ODR_ROAD_MARK_COLOR_RED,
        ODR_ROAD_MARK_COLOR_WHITE,
        ODR_ROAD_MARK_COLOR_GREEN,
        ODR_ROAD_MARK_COLOR_BLUE,
        ODR_ROAD_MARK_COLOR_ORANGE
    };
    
    // LANE TYPE
    enum EnLaneType {
        ODR_LANE_TYPE_NONE = 0,
        ODR_LANE_TYPE_DRIVING,
        ODR_LANE_TYPE_STOP,
        ODR_LANE_TYPE_SHOULDER,
        ODR_LANE_TYPE_BIKING,
        ODR_LANE_TYPE_SIDEWALK,
        ODR_LANE_TYPE_BORDER,
        ODR_LANE_TYPE_RESTRICTED,
        ODR_LANE_TYPE_PARKING,
        ODR_LANE_TYPE_MWY_ENTRY,
        ODR_LANE_TYPE_MWY_EXIT,
        ODR_LANE_TYPE_SPECIAL1,
        ODR_LANE_TYPE_SPECIAL2,
        ODR_LANE_TYPE_SPECIAL3,
        ODR_LANE_TYPE_SPECIAL4,
        ODR_LANE_TYPE_DRIVING_ROADWORKS,
        ODR_LANE_TYPE_TRAM,
        ODR_LANE_TYPE_RAIL,
        ODR_LANE_TYPE_BIDIRECTIONAL,
        ODR_LANE_TYPE_MEDIAN,
        ODR_LANE_TYPE_ENTRY,
        ODR_LANE_TYPE_EXIT,
        ODR_LANE_TYPE_ON_RAMP,
        ODR_LANE_TYPE_OFF_RAMP,
        ODR_LANE_TYPE_CONNECTING_RAMP,
        ODR_LANE_TYPE_BUS,
        ODR_LANE_TYPE_TAXI,
        ODR_LANE_TYPE_HOV 
    };

    // OBJECT TYPE
    enum EnObjectType {
        ODR_OBJECT_TYPE_NONE = 0,
        ODR_OBJECT_TYPE_OBSTACLE,
        ODR_OBJECT_TYPE_WIND,
        ODR_OBJECT_TYPE_POLE,
        ODR_OBJECT_TYPE_TREE,
        ODR_OBJECT_TYPE_VEGETATION,
        ODR_OBJECT_TYPE_BARRIER,
        ODR_OBJECT_TYPE_BUILDING,
        ODR_OBJECT_TYPE_PARKING_SPACE,
        ODR_OBJECT_TYPE_PATCH,
        ODR_OBJECT_TYPE_RAILING,
        ODR_OBJECT_TYPE_TRAFFIC_ISLAND,
        ODR_OBJECT_TYPE_CROSSWALK,
        ODR_OBJECT_TYPE_STREETLAMP,
        ODR_OBJECT_TYPE_GANTRY,
        ODR_OBJECT_TYPE_SOUND_BARRIER,
    };
    
    // DIRECTIONS
    enum EnDirection {
        ODR_DIRECTION_PLUS = 0,
        ODR_DIRECTION_MINUS,
        ODR_DIRECTION_SAME,
        ODR_DIRECTION_OPPOSITE,
        ODR_DIRECTION_NONE
    };
    
    // SIDES
    enum EnSide {
        ODR_SIDE_BOTH = 0,
        ODR_SIDE_LEFT,
        ODR_SIDE_RIGHT,
        ODR_SIDE_UNDEFINED
    };
    
    // ACCESS RESTRICTION
    enum EnAccessRestriction {
        ODR_LANE_ACCESS_RESTRICT_NONE = 0,
        ODR_LANE_ACCESS_RESTRICT_SIMULATOR,
        ODR_LANE_ACCESS_RESTRICT_AUTONOMOUS,
        ODR_LANE_ACCESS_RESTRICT_PEDESTRIAN,
        ODR_LANE_ACCESS_RESTRICT_PASSENGER_CAR,
        ODR_LANE_ACCESS_RESTRICT_BUS,
        ODR_LANE_ACCESS_RESTRICT_DELIVERY,
        ODR_LANE_ACCESS_RESTRICT_EMERGENCY,
        ODR_LANE_ACCESS_RESTRICT_TAXI,
        ODR_LANE_ACCESS_RESTRICT_THROUGH_TRAFFIC,
        ODR_LANE_ACCESS_RESTRICT_TRUCK,
        ODR_LANE_ACCESS_RESTRICT_BICYCLE,
        ODR_LANE_ACCESS_RESTRICT_MOTORCYCLE,
        ODR_LANE_ACCESS_RESTRICT_ALLOW,
        ODR_LANE_ACCESS_RESTRICT_DENY
    };
    
    // TUNNEL TYPES
    enum EnTunnelType {
        ODR_TUNNEL_NONE = 0,
        ODR_TUNNEL_STANDARD,
        ODR_TUNNEL_UNDERPASS
    };
    
    // BRIDGE TYPES
    enum EnBridgeType {
        ODR_BRIDGE_NONE = 0,
        ODR_BRIDGE_CONCRETE,
        ODR_BRIDGE_STEEL,
        ODR_BRIDGE_BRICK
    };

    // COUNTRY CODES
    enum EnCountryCode {
        ODR_COUNTRY_OPENDRIVE = 0,
        ODR_COUNTRY_USA = 1,
        ODR_COUNTRY_FRANCE = 33,
        ODR_COUNTRY_GERMANY = 49
    };

    // MATERIAL CODES
    enum EnMaterialCode {
        ODR_MATERIAL_DEFAULT = 0,
        ODR_MATERIAL_SNOW
    };
    
    // RAILROAD SWITCH STATES
    enum EnRailroadSwitchState {
        ODR_RAILROAD_SWITCH_STATE_STRAIGHT = 0,
        ODR_RAILROAD_SWITCH_STATE_TURN
    };
    
    // UNITS
    enum EnUnit {
        ODR_UNIT_DEFAULT = 0,
        ODR_UNIT_DIST_METER,
        ODR_UNIT_DIST_KILOMETER,
        ODR_UNIT_DIST_FEET,
        ODR_UNIT_DIST_LANDMILE,
        ODR_UNIT_SPEED_MPS,
        ODR_UNIT_SPEED_KMH,
        ODR_UNIT_SPEED_MPH,
        ODR_UNIT_MASS_KG ,
        ODR_UNIT_MASS_TON,
        ODR_UNIT_SLOPE_PERCENT
    };

    // ROAD MARK RULES
    enum EnRoadMarkRule {
        ODR_ROAD_MARK_RULE_DEFAULT = 0,
        ODR_ROAD_MARK_RULE_NO_PASSING,
        ODR_ROAD_MARK_RULE_CAUTION,
        ODR_ROAD_MARK_RULE_NONE
    };

    // OUTLINE_FILL_TYPES
    enum EnOutlineFillType {
        ODR_OUTLINE_FILL_TYPE_DEFAULT = 0,
        ODR_OUTLINE_FILL_TYPE_GRASS,
        ODR_OUTLINE_FILL_TYPE_CONCRETE,
        ODR_OUTLINE_FILL_TYPE_COBBLE,
        ODR_OUTLINE_FILL_TYPE_ASPHALT,
        ODR_OUTLINE_FILL_TYPE_PAVEMENT,
        ODR_OUTLINE_FILL_TYPE_GRAVEL,
        ODR_OUTLINE_FILL_TYPE_SOIL
    };

} // namespace OpenDRIVE

#endif /* _OPENDRIVE_HH */
