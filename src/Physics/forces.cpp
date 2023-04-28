#include "forces.h"
#include <algorithm>


Vec2 Forces::DragForce(const Particle &particle, float k, float density, float Area, float dt){
    
    Vec2 fdrag = Vec2(0,0);
    if ( particle.velocity.MagnitudeSquared() > 0 ){

        float speedSquared = particle.velocity.MagnitudeSquared();
        float speed = particle.velocity.Magnitude();
        Vec2 vDirection = particle.velocity.UnitVector() * -1;
        
        float drag_Magnitude = 0.5 * density * k * speedSquared * Area ;
        float clamped_drag = std::clamp(drag_Magnitude, 0.f, (speed * (1/dt) * 0.99f) );
        fdrag = vDirection * clamped_drag;
    }   

    return fdrag;
}

Vec2 Forces::Friction(const Particle &particle, float kDrag){
        
    Vec2 vDirection = particle.velocity.UnitVector() * -1;
    Vec2 f_friction = vDirection * kDrag;

    return f_friction;
}



Vec2 Forces::DragForceBody(const Body &body, float k, float density, float Area, float dt){
    
    Vec2 fdrag = Vec2(0,0);
    if ( body.velocity.MagnitudeSquared() > 0 ){

        float speedSquared = body.velocity.MagnitudeSquared();
        float speed = body.velocity.Magnitude();
        Vec2 vDirection = body.velocity.UnitVector() * -1;
        
        float drag_Magnitude = 0.5 * density * k * speedSquared * Area ;
        float clamped_drag = std::clamp(drag_Magnitude, 0.f, (speed * (1/dt) * 0.99f) );
        fdrag = vDirection * clamped_drag;
    }   

    return fdrag;
}

Vec2 Forces::FrictionBody(const Body &body, float kDrag){
        
    Vec2 vDirection = body.velocity.UnitVector() * -1;
    Vec2 f_friction = vDirection * kDrag;

    return f_friction;
}

