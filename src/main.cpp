/* #include <hello_world.h> */
#include <flecs.h>
#include <iostream>
#include "raylib-cpp.hpp"
#include "btBulletDynamicsCommon.h"

struct Cube {
    btRigidBody* body;
    Model model;
};

btDiscreteDynamicsWorld* initBullet() {
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
	btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
	btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
	dynamicsWorld->setGravity(btVector3(0, -10, 0));
	return dynamicsWorld;
}

void render(btRigidBody* body, Model model) {
    Color color = WHITE;
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

    DrawModelEx(model, position, axis, angle, {1,1,1}, color);
    DrawModelWiresEx(
    	model, position, axis, angle, {1,1,1},
    	{(unsigned char)(wireColor.r/2), (unsigned char)(wireColor.g/2), (unsigned char)(wireColor.b/2), color.a}
    );
}


int main(int, char *[]) {
    raylib::Color textColor = raylib::Color::LightGray();
    raylib::Window window(1200, 800, "raylib [core] example - basic window");
    SetTargetFPS(60);
    raylib::Camera camera({ 10.0f, 10.0f, 10.0f }, { 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, 45.0f);
    flecs::world ecs;

    btDiscreteDynamicsWorld* dynamicsWorld = initBullet();
    dynamicsWorld->setGravity(btVector3(0, -10, 0));

    ecs.system("BeforeDraw")
        .kind(flecs::PreUpdate)
        .iter([&](flecs::iter& it) {
            /* printf("Time: %f\n", it.delta_time()); */
		    dynamicsWorld->stepSimulation(1.f / 60.f, 10);

            camera.Update(CAMERA_FIRST_PERSON);
            BeginDrawing();
            ClearBackground(DARKBLUE);
            BeginMode3D(camera);
            DrawGrid(100, 1.0f);
        });

    ecs.system<Cube>("Draw")
        .kind(flecs::OnUpdate)
        .each([](Cube& c) {
            render(c.body, c.model);
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

    {
        btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(1.), btScalar(1.), btScalar(1.)));
        btTransform groundTransform;
        groundTransform.setIdentity();
        groundTransform.setOrigin(btVector3(0, 20, 0));
        btScalar mass(1.);
        bool isDynamic = (mass != 0.f);
        btVector3 localInertia(0, 0, 0);
        if (isDynamic) groundShape->calculateLocalInertia(mass, localInertia);
        btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);
        dynamicsWorld->addRigidBody(body);

        Model model = LoadModelFromMesh(GenMeshCube(2,2,2));
        flecs::entity cube1 = ecs.entity("Cube1").set(Cube{body, model});
    }

    while (!window.ShouldClose()) {
    	ecs.progress();
    }
}
