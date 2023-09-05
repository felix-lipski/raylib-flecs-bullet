#include <flecs.h>
#include <iostream>
#include <cmath>
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

enum PlayerState {
    STANDING,
    WALKING,
    JUMPING,
    FALLING
};

struct Player {
    btRigidBody* capsule;
    float height;
    float radius;
    float innerHeight;
    PlayerState state;
};

struct PlayerHead {
    raylib::Camera* camera;
    Vector3 cameraDirection;
};


flecs::entity initPlayer(flecs::world& ecs, btVector3 position) {
    float height = 1.8;
    float radius = 0.25;
    float innerHeight = height - radius - radius;
    btVector3 centerPosition = position + btVector3(0, height/2, 0);

    btCollisionShape* collisionShape = new btCapsuleShape(btScalar(radius), btScalar(innerHeight));
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(centerPosition);
    btScalar mass(1.);
    btVector3 localInertia(0, 0, 0);
    collisionShape->calculateLocalInertia(mass, localInertia);
    btDefaultMotionState* myMotionState = new btDefaultMotionState(transform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, collisionShape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);
    btVector3 angularFactor(0, 1, 0);
    body->setAngularFactor(angularFactor);
    ecs.get<PhysicsWorld>()->dynamicsWorld->addRigidBody(body);

    /* raylib::Camera *camera = new raylib::Camera({ .0f, .0f, 0.0f }, { .0f, .0f, .0f }, { .0f, .0f, .0f }, 70.0f); */
    /* raylib::Camera *camera = new raylib::Camera({ 10.0f, 2.0f, 10.0f }, { 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, 45.0f); */
    /* raylib::Camera *camera = new raylib::Camera({ position.x(), position.y() + (height - radius), position.z() }, { position.x(), position.y() + (height - radius), position.z() + 2.0f}, { 0.0f, 1.0f, 0.0f }, 45.0f); */
    /* raylib::Camera *camera = new raylib::Camera({ position.x(), position.y() + (height - radius), position.z() }, { position.x(), position.y() + (height - radius), position.z() + 2.0f}, { 0.0f, 1.0f, 0.0f }, 45.0f); */
    raylib::Camera *camera = new raylib::Camera({ position.x(), position.y() + (height - radius), position.z() }, { position.x(), position.y() + (height - radius), position.z() + 2.0f}, { 0.0f, 1.0f, 0.0f }, 70.0f);
    /* raylib::Camera *camera = new raylib::Camera({ 0, 0, 0 }, { position.x(), position.y() + (height - radius), position.z() + 2.0f}, { 0.0f, 1.0f, 0.0f }, 70.0f); */
    /* ecs.entity().set(PlayerHead{&camera}); */
    Vector3 direction = {0., 0., 1.0};
    PlayerState state = STANDING;

    return ecs.entity()
        .set(Player{body, height, radius, innerHeight, state})
        .set(PlayerHead{camera,direction}) ;
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
    btScalar mass(10.);
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

// Convert raylib Vector3 to Bullet btVector3
btVector3 toBullet(const Vector3& vec) {
  return btVector3(vec.x, vec.y, vec.z);
}

// Convert Bullet btVector3 to raylib Vector3
Vector3 toRaylib(const btVector3& vec) {
  return Vector3{vec.x(), vec.y(), vec.z()};
}



int main(int, char *[]) {
    raylib::Color textColor = raylib::Color::LightGray();
    /* raylib::Window window(1000, 500, "raylib [core] example - basic window"); */
    raylib::Window window(1920, 1080, "raylib [core] example - basic window");
    SetTargetFPS(60);
    DisableCursor();
    /* ToggleFullscreen(); */

    flecs::world ecs;
    btDiscreteDynamicsWorld* dynamicsWorld = initBullet();
    ecs.set(PhysicsWorld{dynamicsWorld});

    ecs.system<Player, PlayerHead>("Movement")
        .kind(flecs::OnUpdate)
        .each([](Player& player, PlayerHead& ph) {
            // CAMERA
            btVector3 centerPos = player.capsule->getCenterOfMassPosition();
            ph.camera->position.x = centerPos.x();
            ph.camera->position.y = centerPos.y() + (player.innerHeight/2);
            ph.camera->position.z = centerPos.z();

            ph.camera->target.x = centerPos.x() + ph.cameraDirection.x;
            ph.camera->target.y = centerPos.y() + ph.cameraDirection.y + (player.innerHeight/2);
            ph.camera->target.z = centerPos.z() + ph.cameraDirection.z;

            // head rotation
            Vector2 mouseDelta = GetMouseDelta();
            ph.cameraDirection = Vector3RotateByAxisAngle(ph.cameraDirection, Vector3{0., -1.0, 0.}, mouseDelta.x / 100);
            Vector3 perp = Vector3Normalize( Vector3CrossProduct(ph.cameraDirection, Vector3{0., 1.0, 0.}) );
            ph.cameraDirection = Vector3RotateByAxisAngle(ph.cameraDirection, perp, - mouseDelta.y / 100);
            ph.cameraDirection = Vector3Normalize(ph.cameraDirection);

            /* printf("%f, %f\n", mouseDelta.x, mouseDelta.y); */
            /* printf("%f\n",ph.camera->position.y); */

            // RUNNING
            Vector3 desiredDirection = Vector3{0., 0., 0.};

            if (IsKeyDown(KEY_E)) desiredDirection = Vector3Add(desiredDirection, Vector3Scale(ph.cameraDirection, 1.0));
            if (IsKeyDown(KEY_D)) desiredDirection = Vector3Add(desiredDirection, Vector3Scale(ph.cameraDirection, -1.0));
            if (IsKeyDown(KEY_F)) desiredDirection = Vector3Add(desiredDirection, Vector3Scale(perp, 1.0));
            if (IsKeyDown(KEY_S)) desiredDirection = Vector3Add(desiredDirection, Vector3Scale(perp, -1.0));

            desiredDirection = Vector3Scale(desiredDirection, 100.0);
            btVector3 btDesiredDirection = toBullet(desiredDirection);
            btDesiredDirection.normalize();
            player.capsule->applyCentralForce(btVector3(btScalar(desiredDirection.x), btScalar(0.), btScalar(desiredDirection.z)));

            btVector3 linVel = player.capsule->getLinearVelocity();
            btScalar currentY = linVel.y();
            linVel.setY(0.0);
            float maxSpeed = 4.0;
            if (linVel.length() > maxSpeed) {
                btVector3 vel = linVel.normalized() * maxSpeed;
                vel.setY(currentY);
                player.capsule->setLinearVelocity(vel);
            }

        });

    ecs.system<Player>("Jumping")
        .kind(flecs::OnUpdate)
        .each([&](Player& player) {
            btVector3 from = player.capsule->getCenterOfMassPosition();
			btVector3 to(from.x(), from.y() - 1, from.z());

			btCollisionWorld::ClosestRayResultCallback closestResults(from, to);

            ecs.get<PhysicsWorld>()->dynamicsWorld->rayTest(from, to, closestResults);

            if (closestResults.hasHit() && IsKeyPressed(KEY_SPACE)) {
                player.capsule->applyCentralImpulse(btVector3(btScalar(0.), btScalar(6.), btScalar(0.)));
            }
        });

    ecs.system<PhysicsWorld>("StepPhysics")
        .kind(flecs::PreUpdate)
        .each([](PhysicsWorld& pw) {
            pw.dynamicsWorld->stepSimulation(1.f / 60.f, 10);
        });

    ecs.system<PlayerHead>("BeginDrawing")
        .kind(flecs::PreUpdate)
        .each([](PlayerHead& ph) {
            BeginDrawing();
            ClearBackground(DARKBLUE);
            /* ph.camera->Update(CAMERA_FIRST_PERSON); */
            ph.camera->BeginMode();
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
            /* renderPlayer(); */
            btVector3 centerPos = p.capsule->getCenterOfMassPosition();
            /* DrawCapsuleWires( */
            /*         (Vector3){centerPos.x(), centerPos.y() - (p.innerHeight/2), centerPos.z()}, */
            /*         (Vector3){centerPos.x(), centerPos.y() + (p.innerHeight/2), centerPos.z()}, */
            /*         p.radius, 8, 8, PURPLE); */

            /* DrawCapsuleWires((Vector3){0.0f, p.radius, 0.0f}, (Vector3){0.0f, p.radius + p.innerHeight, 0.0f}, p.radius, 8, 8, GREEN); */
        });

    ecs.system("AfterDraw")
        .kind(flecs::PostUpdate)
        .iter([](flecs::iter& it) {
            EndMode3D();
            EndDrawing();
        });


    {
        // TODO free shapes automatically
        btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(100.), btScalar(10.), btScalar(100.)));
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

        Model model = LoadModelFromMesh(GenMeshCube(0,0,0));
        flecs::entity cube1 = ecs.entity("Ground").set(Cube{body, model});
    }

    createCube(ecs, btVector3(1, 15, 1));
    createCube(ecs, btVector3(-1, 17, 0));
    createCube(ecs, btVector3(-2, 1, 0));
    createCube(ecs, btVector3(0, 6, 0));
    initPlayer(ecs, btVector3(0, 0, 0));

    while (!window.ShouldClose()) {
    	ecs.progress();
    }
}
