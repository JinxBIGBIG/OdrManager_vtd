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
 * VIRES Simulationstechnologie GmbH, Oberaustrasse 34, 83026 Rosenheim
 * Germany, opendrive@vires.com, www.vires.com
 */
/* ===================================================
 *  file:       OdrGeoCoord.hh
 * ---------------------------------------------------
 *  purpose:	container for a co-ordinate
 * ---------------------------------------------------
 *  first edit:	28.08.2014 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_GEO_COORD_HH
#define _OPENDRIVE_GEO_COORD_HH

// ====== INCLUSIONS ======

namespace OpenDrive
{

class GeoCoord
{
    public:
        static const double cDegPerM;   // convert degrees in m for perfect sphere

    private:
        /**
        * co-ordinate values:
        *   long, lat, z, heading, pitch, roll
        */
        double  mLong;      // longitude in [deg]
        double  mLat;       // latitude in [deg]
        double  mZ;         // [m]
        double  mH;         // [rad]
        double  mP;         // [rad]
        double  mR;         // [rad]

    public:
        /**
        * primitive constructor
        */
        explicit GeoCoord();

        /**
        * constructor
        *   @param  longitude  longitude value
        *   @param  latitude   latitude value
        *   @param  z   z-value
        *   @param  h   heading
        *   @param  p   pitch
        *   @param  r   roll
        */
        explicit GeoCoord( const double & longitude, const double & latitude, const double & z = 0.0,
                           const double & h = 0.0, const double & p = 0.0, const double & r = 0.0 );

        /**
        * destructor
        */
        virtual ~GeoCoord();
        
        /**
        * overload operator for value assignment
        * @param rhs    coord for assignment
        */
        void operator= ( const GeoCoord & rhs );
        
        /**
        * overload operator for scalar multiplication
        * @param rhs    scalar for multiplication
        */
        GeoCoord operator* ( const double & rhs );
        
        /**
        * add two co-ordinates
        * @param rhs    second co-ordinate
        */
        GeoCoord operator+ ( const GeoCoord & rhs );
        
        /**
        * subtract two co-ordinates
        * @param rhs    second co-ordinate
        */
        GeoCoord operator- ( const GeoCoord & rhs );
        
        /**
        * increase co-ordinate
        * @param rhs    second co-ordinate
        */
        void operator+= ( const GeoCoord & rhs );
        
        /**
        * increase co-ordinate
        * @param rhs    second co-ordinate
        */
        void operator-= ( const GeoCoord & rhs );
        
        /**
        * member access function
        *   @return component of the co-ordinate
        */
        const double & getLong() const;
        
        /**
        * member access function
        *   @return component of the co-ordinate
        */
        const double & getLat() const;
        
        /**
        * member access function
        *   @return component of the co-ordinate
        */
        const double & getZ() const;
        
        /**
        * member access function
        *   @return component of the co-ordinate
        */
        const double & getH() const;
        
        /**
        * member access function
        *   @return component of the co-ordinate
        */
        const double & getP() const;
        
        /**
        * member access function
        *   @return component of the co-ordinate
        */
        const double & getR() const;
        
        /**
        * set values of all member variable
        *   @param xyz and hpr values
        */
        void set( const double & longitude, const double & latitude, const double & z,
                  const double & h = 0.0, const double & p = 0.0, const double & r = 0.0 );
        
        /**
        * set value of member variable
        *   @param component of the co-ordinate
        */
        void setLong( const double & value );
        
        /**
        * set value of member variable
        *   @param component of the co-ordinate
        */
        void setLat( const double & value );
        
        /**
        * set value of member variable
        *   @param component of the co-ordinate
        */
        void setZ( const double & value );
        
        /**
        * set value of member variable
        *   @param component of the co-ordinate
        */
        void setH( const double & value );
        
        /**
        * set value of member variable
        *   @param component of the co-ordinate
        */
        void setP( const double & value );
        
        /**
        * set value of member variable
        *   @param component of the co-ordinate
        */
        void setR( const double & value );

        /**
        * initialize co-ordinate
        */
        void init();

        /**
        * print a co-ordinaate to stderr
        */
        void print() const;

};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_COORD_HH */
