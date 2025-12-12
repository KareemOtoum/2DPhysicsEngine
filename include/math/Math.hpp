// Math.hpp, created by Andrew Gossen.
// Holds math utility functions / helper functions for the linear algebra required in this engine.

#pragma once
#include "core/Vector2.hpp"
#include <cmath>

namespace vecMath{ // Just to avoid potential conflict issues 

    inline float pi=3.141592653589793;

    inline float lengthSquared(const Vec2& a){
        return (a.x*a.x + a.y*a.y);
    }
    inline float length(const Vec2& a){ // Returns the magnitude of a vector 
        return std::sqrt(a.x*a.x + a.y*a.y);
    }

    inline float distanceSquared(const Vec2& a,const Vec2& b){
        float xDis=std::abs(a.x-b.x);
        float yDis=std::abs(a.y-b.y);
        return (xDis*xDis + yDis*yDis);
    }

    inline float distance(const Vec2& a, const Vec2& b){ // Returns the distance between two vectors 
        float xDis=std::abs(a.x-b.x);
        float yDis=std::abs(a.y-b.y);
        return std::sqrt(xDis*xDis + yDis*yDis);
    }

    inline float dot(const Vec2& a,const Vec2& b) { // Dot product of two vectors ( Magnitude of a component in direction of vector B )
        return (a.x*b.x)+(a.y*b.y);
    }

    inline float cross(const Vec2& a, const Vec2& b) { // Cross product of two vectors ( Perpendicular vector to both a and b )
        //  Returns solely the Z component, as cross priduct is scalar in 2 dimensions
        return (a.x*b.y)-(a.y*b.x);
    }

    inline Vec2 floatCross(float s, const Vec2& v) {
        return Vec2(-s * v.y, s * v.x);
    }

    inline bool floatCloselyEqual(float a,float b){
        return std::abs(a-b) < (1e-3);  // Half mm precision 
    }

    inline bool vecCloselyEqual(const Vec2& a, const Vec2&b){
        return (floatCloselyEqual(a.x,b.x) && floatCloselyEqual(a.y,b.y));
    }

    inline float pointSegmentDistance(const Vec2& a,const Vec2& b,const Vec2& p,Vec2& contactValue){

        Vec2 ab = (b - a); // segment AB
        Vec2 ap = (p - a); // from A to P 

        float abLengthSquared = lengthSquared(ab);
        if (abLengthSquared <= 0.0f) {
            contactValue = a;
            return distanceSquared(p, a);
        }

        float t = dot(ap, ab) / abLengthSquared;

        Vec2 contact;
        if (t <= 0.0f){
            contact = a;
        } else if (t >= 1.0f){
            contact = b;
        } else { 
            contact = a + ab * t;
        }

        contactValue = contact;
        return distanceSquared(p, contact);

    }

} // namespace vecMath
