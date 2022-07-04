/*
 * Copyright 2019 VIRES Simulationstechnologie GmbH
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
 *  file:       OdrManager.hh
 * ---------------------------------------------------
 *  purpose:	manager for the interaction with
 *              the OpenDRIVE routines
 * ---------------------------------------------------
 *  first edit:	08.07.2006 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_ODRMANAGER_HH
#define _OPENDRIVE_ODRMANAGER_HH

// ====== INCLUSIONS ======
#include <map>
#include "OdrManagerLite.hh"
#include "OdrGeoCoord.hh"

namespace OpenDrive
{
    
// forward declarations of unpublished classes
class ParserCallback;
class Node;
class RoadHeader;
class LaneSection;
class Lane;
class Elevation;
class Superelevation;
class Path;
class JuncHeader;
class RoadMark;

class OdrManager : public OdrManagerLite
{
    // static methods
    public:
        /**
        * delete the current position object
        * @param position pointer to a valid position object
        */
        static void deletePosition( Position* & position );
    
    public:
        // some methods of base class must remain accessible even if methods
        // with identical name are defined in this derived class
        using OdrManagerLite::setPos;
        
        static const unsigned int LOADER_ADD_BORDERS;
        static const unsigned int LOADER_VERBOSE;
        static const unsigned int LOADER_XTEND_ADD_SIGNALS;
        static const unsigned int LOADER_SPLIT_JUNCTIONS;
        static const unsigned int LOADER_CHECK_GEOMETRY;
        static const unsigned int LOADER_CONTINUOUS_OBJECT_TESSELATION;
        static const unsigned int LOADER_RESOLVE_REPEATED_OBJECTS;
        static const unsigned int LOADER_IGNORE_SURFACE;
        static const unsigned int LOADER_INHIBIT_CURVATURE_APPROXIMATION;    /** @version 1.5.1 */
    
        /**
        * default constructor
        * @param showIntro enable display of intro text
        */
        explicit OdrManager( bool showIntro = false );

        /**
        * destructor
        */
        virtual ~OdrManager();

        // added by JM ---20211102
        RoadMark* getQueriedRoadMark();
        
        /**
        * show intro text
        */
        virtual void intro();
        
        // new in 1.3.11
        /**
        * load a string containing a valid OpenDRIVE definition
        * @param  odrData  valid  OpenDRIVE definition
        * @return success/failure
        */
        virtual bool loadData( const std::string & odrData );
        
        /**
        * load a file
        * @param  name  filename (complete path)
        * @return success/failure
        */
        virtual bool loadFile( const std::string & name );
        
        /**
        * add a file to already existing data
        * @param  name  filename (complete path)
        * @return success/failure
        */
        virtual bool addFile( const std::string & name );
        
        /**
        * set loader options
        * @param option option type
        * @param value  value for the option
        */
        void setLoaderOption( const unsigned int & option, const double & value = 0.0 );
        
        /**
        * get a new position object
        * @return NULL if no object has been created
        */
        virtual Position* createPosition();

        /**
        * convert track position into lane position
        *   @return true if successful
        */
        bool track2lane();

        /**
        * calculate curvature at given track position
        *   @return true if successful
        */
        bool track2curvature();

        /**
        * convert lane position into track position
        *   @return true if successful
        */
        bool lane2track();

        /**
        * member access function
        * @return track width at current position
        */
        const double & getTrackWidth() const;

        /**
        * member access function
        * @return track angles at current position
        */
        const Coord & getTrackAngles() const;

        /**
        * member access function
        * @return  road mark of current lane
        */
        const unsigned short & getRoadMark() const;
        
        /**
        * trigger parsing of the memory contents
        * with a callback for each node encountered
        * @param cb pointer to a class which can handle node callbacks
        */
        void parseTree( ParserCallback *cb );
        
        /**
        * access the root node of the OpenDRIVE data tree
        * @return pointer to root node
        */
        Node* getRootNode();
        const Node* getRootNode() const;

        /**
        * convert inertial position into track position and attach to track
        *   @return true if successful
        */
        virtual bool inertial2track();

        /**
        * convert inertial position into lane position and calculate foot point
        * @return true if successful
        */
        virtual bool inertial2lane();

        /**
        * convert inertial position into lane position and calculate foot point
        * tries to find the position on the given track only
        * @param trackId    id of the track on which to search for the position
        * @return true if successful
        */
        bool inertial2lane( int trackId );

        /**
        * convert inertial position into lane position and calculate foot point
        * tries to find the position on the given track only
        * @param trackId    id of the track on which to search for the position
        * @return true if successful
        * @version 1.4.5
        */
        bool inertial2lane( const std::string& trackId );

        /**
        * convert inertial position into a list of possible lane position
        * (foot point only); use getLaneList() to evaluate the result
        * @param  fullSearch  search entire database, omitting internal optimizations
        * @return true if successful
        */
        bool inertial2laneList( bool fullSearch = false );

        /**
        * convert inertial position into geo-referenced position
        * @return true if successful
        * @version 1.3.13
        */
        bool inertial2geo();

        /**
        * convert geo-referenced position into inertial position
        * @return success(true) / false(failure)
        * @version 1.3.13
        */
        bool geo2inertial();

        /**
        * get access to the resulting lane vector from previous query
        * @return reference to lane list
        */
        LaneCoord::LaneVec & getLaneList();
        
        /**
        * get the resulting node of the last query, provided that
        * the query was successful
        */
        RoadHeader*     getRoadHeader();
        LaneSection*    getLaneSection();
        Lane*           getLane();
        Elevation*      getElevation();
        Superelevation* getSuperelevation();
        
        /**
        * get the ID of the junction, if position is on a track
        * belonging to a junction
        * @return ID of the junction or -1 if none
        */
        int getJunctionId();
        
        /**
        * get the type of the lane, if position is on a valid lane
        * @return ID of the lane type or ODR_LANE_TYPE_NONE
        */
        unsigned int getLaneType();
               
        /**
        * get the material parameters at the given lane position
        * @param surface   surface code (return value)
        * @param friction  surface friction value (return value)
        * @param roughness surface roughness value (return value)
        * @return true if valid material entry was found
        * @version 1.4.21
        */
        bool getMaterial( std::string & surface, double & friction, double & roughness );
        
        /**
        * get the material parameters at the given lane position
        * @param code      surface code (return value)
        * @param friction  surface friction value (return value)
        * @param roughness surface roughness value (return value)
        * @return true if valid material entry was found
        */
        bool getMaterial( unsigned int & code, double & friction, double & roughness );
        
        /**
        * intersect the road network with a circle drawn around the current
        * inertial position and return a list of intersected roads
        * (foot point only); use getLaneList() to evaluate the result
        * @param  radius radius of the circle
        * @return true if successful
        */
        bool intersectCircle( const double & radius );
        
        /**
        * assign the given path to the currently active position object
        * @param  path  pointer to path object
        */
        void assignPath( Path* path );
        
        /**
        * print the given path
        * @param  path  pointer to path object
        */
        void printPath( Path* path );

        /**
        * convert lane position into path position
        *   @return true if successful
        */
        bool lane2path();

        /**
        * convert inertial position into path position
        *   @return true if successful
        */
        bool inertial2path();
        
        /**
        * add a delta lane position to the current path position
        * @param ds         delta in s direction
        * @param dLane      lane change 
        * @param dLaneOff   change of the lane offset
        * @return   true if successful
        */
        bool addPathOffset( const double & ds, const int & dLane, const double & dLaneOff ); 
        
        /**
        * create a new path object
        * @param name   name of the path
        * @return pointer to path object
        */
        Path* createPath( const std::string & name );
        
        /**
        * delete path object
        * @param path   pointer to path object
        */
        void deletePath( Path* path );          // new in 1.1.27
        
        /**
        * add the given position to the current position object's path
        * objects and trigger its re-calculation
        * @param  trackPos new end position of the path
        * @return true if successful
        */
        bool addPosToPath( const TrackCoord & trackPos ); 
        
        /**
        * set the path position (along lane 0) directly
        * @param  s new path position [m]
        * @return true if successful
        */
        bool setPathPos( const double & s );
        
        /**
        * get the length of the active position object's path
        * @return length in [m]
        */
        double getPathLength();        
        
        /**
        * get the position on the active position object's path
        * @return progressive co-ordinate s
        */
        double getPathPos();
        
        /**
        * get the direction of the road relative to the path at the current
        * position
        * @return true if underlying road points into positive path direction
        */
        bool getPathFwd();
        
        /**
        * copy the position from one position object to another without having
        * to change the active position
        * @param dst    destination position
        * @param src    source position
        */
        void copyPos( Position* dst, Position* src );

        // new in 1.1.15
        /**
        * trigger calculation of actual lane curvature at current lane position.
        * This takes into account any "skewing" of the lane by width changes
        * of inner lanes etc.
        *  @return lane curvature (0.0 if no curvature could be calculated)
        */
        double getLaneCurvature();

        /**
        * get a list of signals between current position and maximum dist. in the given road direction
        * @param fwd if true, search in positive track direction (default)
        * @param maxDist maximum distance to search
        * @return true if signals have been found
        */
        bool collectSignals( bool fwd = true, const double & maxDist = 1.0e10 );

        /**
        * get the number of nodes in the most recent collection
        * @return number of entries in collection
        */
        int getCollectionSize();

        /**
        * get the information about an element in the most recent collection
        * @param id id of the element that is to be queried
        * @param dist       distance to signal
        * @param node       data node (e.g. signal)
        * @return success/failure
        */
        bool getCollectionInfo( const int & id, double & dist, Node* & node );
        
        // new in 1.1.18
        /**
        * convert the current track position exceeding the current track
        * into a valid track position (i.e. follow the link information)
        * @return true if successful
        */
        bool track2validTrack();

        /**
        * convert the current lane position exceeding the current lane
        * into a valid lane position (i.e. follow the link information)
        * @return true if successful
        */
        bool lane2validLane();

        /**
        * trigger calculation of track angle derivatives at current position
        * @return true if successful
        */
        bool calcTrackAnglesDot();
        
        /**
        * get several derivatives of angles calculated at a given track position
        * @return the corresponding value
        */
        const double & getDhDs() const;
        const double & getDpDs() const;
        const double & getDrDs() const;
        const double & getDzDs() const;
        const double & getD2zDs() const;
        const double & getDzDt() const;
        
        /**
        * collect information about the entire cross section of the road
        * @return success/failure
        */
        bool calcCrossSection();

        /**
        * get the number of lanes in the cross section
        * @return number of entries in cross section information
        */
        int getCrossSectionSize();

        /**
        * get the information about an element in the cross section
        * @param id id of the element that is to be queried
        * @param laneId     id of the lane
        * @param laneType   type of the lane
        * @param laneWidth  width of the lane
        * @return success/failure
        */
        bool getCrossSectionLaneInfo( const int & id, int & laneId, int & laneType, double & laneWidth );

        /**
        * performance-optimized series of calculations based on track co-ordinate:
        * - track2inertial()
        * - calcTrackAnglesDot()
        * - calcCrossSection()
        *   @return true if successful
        */
        bool track2inertialAngDotCrossSec();
        
        /**
        * get the next junction in the given track direction (requires valid track/lane position)
        * @param fwd        go in forward or backward direction of the track
        * @param jHdr       pointer to junction header (return value, 0 if none is found)
        * @param distToJunc distance to junction along tracks (return value)
        * @return true if successful
        */
        bool getNextJunction( bool fwd, JuncHeader* & jHdr, double & distToJunc );

        /**
        * get the size of the collected junction info vector
        * @return size of result vector
        */
        int getJunctionInfoSize();
        
        /**
        * retrieve information at a given position in the junction query result vector
        * @param index  index where to retrieve the information
        * @param inRoad     pointer to incoming road header (returned)
        * @param connRoad   pointer to connecting road header (returned)
        * @param turnAngle  turning angle between incoming and outgoing direction ( [-PI;PI] -PI..0 = right; 0..PI = left )
        * @param minLane    minimum ID of lane on incoming road leading into the junction
        * @param maxLane    maximum ID of lane on incoming road leading into the junction
        * @return true if successful
        */
        bool getJunctionInfo( const unsigned int & index, RoadHeader* & inRoad, RoadHeader* & connRoad, 
                              double & turnAngle, int & minLane, int & maxLane );
        
        // new in 1.1.19
        /**
        * get the road type at valid lane position
        * @return road type according to specification
        */
        int getRoadType();

        /**
        * get the lane speed at the current lane position
        * @return speed restriction (-1.0 if none was found)
        */
        double getLaneSpeed();
        
        // new in 1.1.20
        /**
        * get the lane curvature's first derivative at current position
        * @return lane curvature's first derivative
        */
        const double & getLaneCurvatureDot() const;

        /**
        * get the vertical lane curvature at current position
        * @return vertical lane curvature
        */
        const double & getLaneCurvatureVert() const;

        /**
        * get the vertical lane curvature's first derivative at current position
        * @return vertical lane curvature's first derivative
        */
        const double & getLaneCurvatureVertDot() const;
        
        /**
        * print the given position
        * @param  pos  pointer to position object
        * @version 1.1.42
        */
        void print( Position* pos );
        
        /**
        * set the surface scale factor for a query
        * @param factor surface scale factor
        * @version 1.3.2
        */
        void setSurfaceScale( const double & factor );

        /**
        * print position info
        * @param ident  number of leading spaces
        * @version 1.3.5
        */
        void print( int ident = 0 );
        
        /**
        * check whether current position is in a tunnel
        * works with valid lane position only
        * @return true if position is in a tunnel
        * @version 1.3.13
        */
        bool inTunnel();
        
        /**
        * check whether current position is on a bridge
        * works with valid lane position only
        * @return true if position is on a bridge
        */
        bool onBridge();
        
        /**
        * member access function
        *   @return geographic co-ordinate
        * @version 1.3.13
        */
        const GeoCoord & getGeoPos() const;

        /**
        * member assignment function
        * @param set the geo-referenced co-ordinate
        * @version 1.3.13
        */
        void setPos( const GeoCoord & value );

        /**
        * explicitly set the geo-referenced position
        * @param longitude  longitude in [deg]
        * @param latitude   latitude in [deg]
        * @param z  z co-ordinate in [m]
        * @version 1.3.13
        */
        void setGeoPos( const double & longitude, const double & latitude, const double & z );
        
        /**
        * set the dimensions of a patch that shall be used in contact point calculations
        * involving detailed surface data (e.g. OpenCRG)
        * applies to the current position object only!
        * @param[in] length length of contact patch [m]
        * @param[in] width  width of contact patch [m]
        * @version 1.4.0
        */
        void setContactPatchDimension( const double & length, const double & width );
        
        /**
        * activate / deactivate the consideration of lane height for total height
        * @param enable if true, lane height will be taken in to account
        * @version 1.4.0
        */
        void useLaneHeight( bool enable = true );
        
        /**
        * set the inertial system's Proj4 string for the projection
        * e.g. "+proj=utm +zone=32 +ellps=WGS84 +datum=WGS84 +units=m +no_defs"
        * @param prjStr Proj4 string
        * @return true if successful
        * @version 1.4.5
        */
        bool projectionSetProj4Inertial( const std::string & prjStr );
        
        /**
        * set the geo system's Proj4 string for the projection
        * e.g. "+proj=latlong +datum=WGS84"                                
        * @param prjStr Proj4 string
        * @return true if successful
        * @version 1.4.5
        */
        bool projectionSetProj4Geo( const std::string & prjStr );
        
        /**
        * set the geo system's OGCWKT string for the projection
        * @param prjStr OGCWKT string
        * @return true if successful
        * @version 1.4.5
        */
        bool projectionSetOGCWKT( const std::string & prjStr );
        
        /**
        * set the origin of the inertial co-ordinate system which is to be taken into account as
        * offset when interpreting / calculating the inertial position
        * @param offset         inertial offset
        * @version 1.4.5
        */
        void setOriginInertial( const Coord & offset );
        
        /**
        * get the origin of the inertial co-ordinate system which is to be taken into account as
        * offset when interpreting / calculating the inertial position
        * @return inertial offset
        * @version 1.4.5
        */
        const Coord & getOriginInertial() const;
        
        /**
        * add OpenCRG surface data to the database
        * @param roadId anchor road
        * @param file filename
        * @param sStart start of OpenCRG area along reference line
        * @param sEnd end of OpenCRG area along reference line
        * @param orientation orientation of OpenCRG data [ODR_DIRECTION_PLUS, ODR_DIRECTION_MINUS]
        * @param mode mapping mode of OpenCRG data [sModeGenuine, sModeAttached, sModeAttached0]
        * @param sOffset offset between s-value (OpenDRIVE domain) and u-value (OpenCRG domain)
        * @param tOffset offset between t-value (OpenDRIVE domain) and v-value (OpenCRG domain)
        * @param zOffset offset between z-value (OpenDRIVE domain) and z-value (OpenCRG domain)
        * @param zScale scale factor for OpenCRG data
        * @param hOffset heading offset between OpenCRG refline and OpenDRIVE refline (for mode "genuine" only)
        * @param purpose purpose of the data contained in OpenCRG file [sPurposeElevation, sPurposeFriction, sPurposeAny]
        * @return true if successful
        * @version 1.4.5
        */
        bool addSurfaceCRG( const unsigned int & roadId, const std::string & file, const double & sStart,
                            const double & sEnd, const unsigned char & orientation, const unsigned int & mode,
                            const double & sOffset, const double & tOffset, const double & zOffset, const double & zScale,
                            const double & hOffset, const unsigned short & purpose );
        
        bool addSurfaceCRG( const std::string & roadId, const std::string & file, const double & sStart,
                            const double & sEnd, const unsigned char & orientation, const unsigned int & mode,
                            const double & sOffset, const double & tOffset, const double & zOffset, const double & zScale,
                            const double & hOffset, const unsigned short & purpose );
        
        
        /**
        * add a signal to the existing database
        * @param xPos        x-position of signal's reference point
        * @param yPos        y-position of signal's reference point
        * @param zPos        z-position of signal's reference point
        * @param hPos        heading of signal
        * @param pPos        pitch of signal
        * @param rPos        roll of signal
        * @param id          alphanumeric ID
        * @param name        name of the sign
        * @param dynamic     if true, sign is a signal
        * @param orientation direction relative to road ("+" or "-")
        * @param country     alphanumeric ODR country code
        * @param type        alphanumeric type description
        * @param subtype     alphanumeric subtype description
        * @param value       float value ot the sign
        * @param state       numeric state
        * @param unit        unit identifier
        * @param height      height
        * @param width       width
        * @param text        alphanumeric text associated with sign
        * @return true if signal could be added
        * @version 1.4.5
        */
        bool addSignal( const double & xPos, const double & yPos, const double & zPos, 
                        const float & hPos,  const float & pPos,  const float & rPos,
                        const std::string & id, const std::string & name, bool dynamic, const std::string & orientation,
                        const std::string & country, const std::string & type, const std::string & subtype,
                        const double & value, unsigned int state, const std::string & unit,
                        const float & height, const float & width, const std::string & text );
        
        /**
        * attach to existing (i.e. already loaded) road data
        * @param true if manager could attach to road data;
        */
        bool attach();
        
        /**
        * activate / deactivate optimization for z position when mapping inertial position to track or lane position
        * @param[in] enable     if true, z-optimization will be activated
        * @param[in] tolerance  maximum deviation that is to be considered a direct hit
        * @version 1.4.6
        */
        void setZOptimization( bool enable = true, const float & tolerance = 2.0f );
        
    private:
        /**
        * hold a map of loader options
        */
        typedef std::map < unsigned int, double > OptionMap;
        OptionMap mOptionMap;
        
        /**
        * optimize for z deviation
        * @version 1.4.6
        */
        bool mMinimizeDeltaZ;
        
        /**
        * z tolerance for optimizing z deviation [m]
        * @version 1.4.6
        */
        float mZTolerance;
                
    private:
        /**
        * actually load the data (either from file or from data string)
        * @param filename   filename which is to be loaded
        * @param odrData    odrData string which is to be loaded
        */
        bool load( const std::string & filename = "", const std::string & odrData = "" ); 
        
        /**
        * add OpenCRG surface data to the database
        * @param rdHdr pointer to road header
        * @param file filename
        * @param sStart start of OpenCRG area along reference line
        * @param sEnd end of OpenCRG area along reference line
        * @param orientation orientation of OpenCRG data [ODR_DIRECTION_PLUS, ODR_DIRECTION_MINUS]
        * @param mode mapping mode of OpenCRG data [sModeGenuine, sModeAttached, sModeAttached0]
        * @param sOffset offset between s-value (OpenDRIVE domain) and u-value (OpenCRG domain)
        * @param tOffset offset between t-value (OpenDRIVE domain) and v-value (OpenCRG domain)
        * @param zOffset offset between z-value (OpenDRIVE domain) and z-value (OpenCRG domain)
        * @param zScale scale factor for OpenCRG data
        * @param hOffset heading offset between OpenCRG refline and OpenDRIVE refline (for mode "genuine" only)
        * @param purpose purpose of the data contained in OpenCRG file [sPurposeElevation, sPurposeFriction, sPurposeAny]
        * @return true if successful
        * @version 1.4.5
        */
        bool addSurfaceCRG( RoadHeader* rdHdr, const std::string & file, const double & sStart,
                            const double & sEnd, const unsigned char & orientation, const unsigned int & mode,
                            const double & sOffset, const double & tOffset, const double & zOffset, const double & zScale,
                            const double & hOffset, const unsigned short & purpose );       
};

} // namespace OpenDRIVE

#endif /* _OPENDRIVE_ODRMANAGER_HH */
