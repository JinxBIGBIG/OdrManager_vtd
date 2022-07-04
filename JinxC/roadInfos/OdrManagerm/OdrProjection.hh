/*
 * Copyright 2014 VIRES Simulationstechnologie GmbH
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
 * VIRES Simulationstechnologie GmbH, Grassinger Strasse 8, 83043 Bad Aibling
 * Germany, opendrive@vires.com, www.vires.com
 */
/* ===================================================
 *  file:       OdrProjection.hh
 * ---------------------------------------------------
 *  purpose:	container for a co-ordinate
 * ---------------------------------------------------
 *  first edit:	28.08.2014 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_PROJECTION_HH
#define _OPENDRIVE_PROJECTION_HH

// ====== INCLUSIONS ======
#include "OdrGeoCoord.hh"
#include "OdrCoord.hh"

namespace OpenDrive
{

class Projection
{
    private:
        /**
        * projection strings
        */
        std::string mOGCWKTString;     // OGC-WKT string
        std::string mProj4StringInertial; // Proj4 string
        std::string mProj4StringGeo;      // Proj4 string
        
        /**
        * several constants
        */
        static const double      cDeg2Rad;
        static const std::string cUTMString;
        static const std::string cWGSString;
        
        /**
        * conversion pointers
        */
        void* mPj_latlong;
        void* mPj_utm;
        
        /**
        * origin of inertial co-ordinate system which may be used for reducing
        * the order of magnitude of co-ordinates computed within the application
        *   InertialCoordForConversion = LocalCoord + mOriginInertial;
        */
        Coord mOriginInertial;

    public:
        /**
        * primitive constructor
        */
        explicit Projection();

        /**
        * destructor
        */
        virtual ~Projection();
        
        /**
        * set the OGC-WKT string
        * @param ogcStr OGC WKT string
        */
        void setOGCWKT( const std::string & ogcStr );
        
        /**
        * set the Proj4 string for the inertial system
        * e.g. "+proj=utm +zone=32 +ellps=WGS84 +datum=WGS84 +units=m +no_defs"
        * @param prjStr Proj4 string
        */
        void setProj4Inertial( const std::string & prjStr );
        
        /**
        * set the Proj4 string for the geo system
        * e.g. "+proj=latlong +datum=WGS84"                                
        * @param prjStr Proj4 string
        */
        void setProj4Geo( const std::string & prjStr );
        
        /**
        * initialize co-ordinate
        */
        void init();

        /**
        * print a co-ordinaate to stderr
        */
        void print() const;
        
        /**
        * convert inertial position into geo position
        * @param inertialPos    inertial position (compensated for non-zero origin)
        * @param geoPos         resulting geo-referenced position
        * @return   result of the query
        * @version 1.4.5
        */
        int inertial2geo( const Coord & inertialPos, GeoCoord & geoPos );

        /**
        * convert geo position into inertial position
        * @param geoPos         geo-referenced position
        * @param inertialPos    resulting inertial position (compensated for non-zero origin)
        * @return   result of the query
        * @version 1.4.5
        */
        int geo2inertial( const GeoCoord & geoPos, Coord & inertialPos );
        
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
};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_PROJECTION_HH */
