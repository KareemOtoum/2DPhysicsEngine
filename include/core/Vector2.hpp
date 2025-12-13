
// Vector2.h, created by Andrew Gossen.

// ----- 
// Lightweight vector2 value type.
//
// Usage:
// - Represents positions, velocities, accelerations, forces, etc.
// - This is a pure value type: no ownership, and no dynamic allocation.
//
// Notes:
// - All operations are inline  and cheap.
// - Equality is exact floating-point comparison (see notes below).
// -----

#pragma once
#include <cmath>

struct Vec2{ 

    float x{0.0f};
    float y{0.0f};
    Vec2()=default;
    Vec2(float x,float y) : x(x), y(y) {}

    // Vector addition
    Vec2 operator +(const Vec2& otherVector) const { // Add this vector with another 
        return Vec2(x+otherVector.x,y+otherVector.y);
    } 

    // Vector subtraction
    Vec2 operator -(const Vec2& otherVector) const { 
        return Vec2(x-otherVector.x,y-otherVector.y);
    } 

    // Scalar multiplier 
    Vec2 operator *(float scalar) const { 
        return Vec2(x*scalar,y*scalar);
    }
     // Scalar division
    Vec2 operator /(float scalar) const {
        return Vec2(x/scalar,y/scalar);
    }

    // Compound addition 
    Vec2& operator +=(const Vec2& otherVector) { 
        x+=otherVector.x; y+=otherVector.y;
        return *this;
    };

    // Compound subtraction
    Vec2& operator -=(const Vec2& otherVector) { 
        x-=otherVector.x; y-=otherVector.y;
        return *this;
    };

    // Compound multiplication
    Vec2& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    // Check whether this vector and otherVector are equivalent, won't be used for physics calculations due to floating point precision 
    bool operator ==(const Vec2& otherVector) const { 
        return ( (x==otherVector.x) && (y==otherVector.y )); 
    }

    // Get magnitude of this vector 
    float length() const{
        return std::sqrt(x*x + y*y);
    }

    float lengthSquared() const{
        return (x*x + y*y);
    }

    // Get unit vector and avoid division by zero
    Vec2 normalise() const{ 
        float len=length();
        if (len > 1e-6f) {
            return Vec2{x / len, y / len};
        }
        return Vec2{0.0f, 0.0f};
    }

};


