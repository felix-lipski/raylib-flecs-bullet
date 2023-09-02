#include <flecs.h>
#include <iostream>
#include "raylib-cpp.hpp"
#include "btBulletDynamicsCommon.h"

btDiscreteDynamicsWorld* initBullet();

struct PhysicsWorld {
    btDiscreteDynamicsWorld* dynamicsWorld;
};

struct Cube {
    btRigidBody* body;
    Model model;
};

struct Player {
    btRigidBody* capsule;
};

float height = 1.8;
float radius = 0.25;
float innerHeight = height - radius - radius;

flecs::entity initPlayer(flecs::world& ecs, btVector3 position) {
    btCollisionShape* collisionShape = new btCapsuleShape(btScalar(radius), btScalar(innerHeight));
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(position);
    btScalar mass(1.);
    btVector3 localInertia(0, 0, 0);
    collisionShape->calculateLocalInertia(mass, localInertia);
    btDefaultMotionState* myMotionState = new btDefaultMotionState(transform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, collisionShape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);
    ecs.get<PhysicsWorld>()->dynamicsWorld->addRigidBody(body);
    return ecs.entity().set(Player{body});
}
void renderPlayer() {
    DrawCapsuleWires((Vector3){0.0f, radius, 0.0f}, (Vector3){0.0f, radius + innerHeight, 0.0f}, radius, 8, 8, PURPLE);
}
/* DrawCapsuleWires((Vector3){-3.0f, 1.5f, -4.0f}, (Vector3){-4.0f, -1.0f, -4.0f}, 1.2f, 8, 8, PURPLE); */

void renderCube(btRigidBody* body, Model model) {
    Color color = ORANGE;
    Color wireColor = BLACK;
    const float radian_scale = 57.296;
    btTransform trans;
    if (body && body->getMotionState()) body->getMotionState()->getWorldTransform(trans); else return;
    Vector3 position = {
    	float(trans.getOrigin().getX()),
    	float(trans.getOrigin().getY()),
    	float(trans.getOrigin().getZ())
    };
    btQuaternion quat = trans.getRotation();
    Vector3 axis = {
    	float(quat.getAxis().getX()),
    	float(quat.getAxis().getY()),
    	float(quat.getAxis().getZ())
    };
    float angle = float( quat.getAngle() ) * radian_scale; // Convert radians to degrees

    /* DrawModelEx(model, position, axis, angle, {1,1,1}, color); */
    DrawModelWiresEx(
    	model, position, axis, angle, {1,1,1},
    	{(unsigned char)(wireColor.r/2), (unsigned char)(wireColor.g/2), (unsigned char)(wireColor.b/2), color.a}
    );
}

flecs::entity createCube(flecs::world& ecs, btVector3 position) {
    btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(1.), btScalar(1.), btScalar(1.)));
    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(position);
    btScalar mass(1.);
    bool isDynamic = (mass != 0.f);
    btVector3 localInertia(0, 0, 0);
    if (isDynamic) groundShape->calculateLocalInertia(mass, localInertia);
    btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);
    ecs.get<PhysicsWorld>()->dynamicsWorld->addRigidBody(body);
    Model model = LoadModelFromMesh(GenMeshCube(2,2,2));
    return ecs.entity().set(Cube{body, model});
}


int main(int, char *[]) {
    raylib::Color textColor = raylib::Color::LightGray();
    raylib::Window window(1920, 1080, "raylib [core] example - basic window");
    SetTargetFPS(60);
    /* ToggleFullscreen(); */
    raylib::Camera camera({ 10.0f, 2.0f, 10.0f }, { 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, 45.0f);
    flecs::world ecs;

    btDiscreteDynamicsWorld* dynamicsWorld = initBullet();
    ecs.set(PhysicsWorld{dynamicsWorld});

    ecs.system("BeforeDraw")
        .kind(flecs::PreUpdate)
        .iter([&](flecs::iter& it) {
            /* printf("Time: %f\n", it.delta_time()); */
		    dynamicsWorld->stepSimulation(1.f / 60.f, 10);

            /* camera.Update(CAMERA_FIRST_PERSON); */
            BeginDrawing();
            ClearBackground(DARKBLUE);
            BeginMode3D(camera);
            DrawGrid(100, 1.0f);
        });

    ecs.system<Cube>("DrawCubes")
        .kind(flecs::OnUpdate)
        .each([](Cube& c) {
            renderCube(c.body, c.model);
        });

    ecs.system<Player>("DrawPlayer")
        .kind(flecs::OnUpdate)
        .each([](Player& p) {
            renderPlayer();
            /* renderCube(c.body, c.model); */
        });

    ecs.system("AfterDraw")
        .kind(flecs::PostUpdate)
        .iter([&](flecs::iter& it) {
            EndMode3D();
            EndDrawing();
        });


    {
        // TODO free shapes automatically
        btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(10.), btScalar(10.), btScalar(10.)));
        btTransform groundTransform;
        groundTransform.setIdentity();
        groundTransform.setOrigin(btVector3(0, -10, 0));
        btScalar mass(0.);
        bool isDynamic = (mass != 0.f);
        btVector3 localInertia(0, 0, 0);
        if (isDynamic) groundShape->calculateLocalInertia(mass, localInertia);
        btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);
        dynamicsWorld->addRigidBody(body);

        Model model = LoadModelFromMesh(GenMeshCube(20,20,20));
        flecs::entity cube1 = ecs.entity("Ground").set(Cube{body, model});
    }

    /* createCube(ecs, btVector3(1, 15, 1)); */
    /* createCube(ecs, btVector3(-1, 17, 0)); */
    /* createCube(ecs, btVector3(-2, 1, 0)); */
    createCube(ecs, btVector3(0, 6, 0));
    initPlayer(ecs, btVector3(0, 2, 0));

    while (!window.ShouldClose()) {
    	ecs.progress();
    }
}
