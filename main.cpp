#include <string>
#include <memory>
#include <fstream>
#include <sstream>

#include <Urho3D/Urho3D.h>
#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Graphics/RenderPath.h>
#include <Urho3D/Engine/Application.h>
#include <Urho3D/Engine/Engine.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/Input/InputEvents.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Resource/XMLFile.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/UI/UI.h>
#include <Urho3D/UI/Text.h>
#include <Urho3D/UI/Text3D.h>
#include <Urho3D/UI/Font.h>
#include <Urho3D/UI/Button.h>
#include <Urho3D/UI/UIEvents.h>
#include <Urho3D/UI/Window.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Scene/SceneEvents.h>
#include <Urho3D/Graphics/Graphics.h>
#include <Urho3D/Graphics/Camera.h>
#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/Graphics/Renderer.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <Urho3D/Graphics/Octree.h>
#include <Urho3D/Graphics/Light.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Skybox.h>
#include <Urho3D/Graphics/Zone.h>
#include <Urho3D/Graphics/RibbonTrail.h>
#include <Urho3D/Audio/Sound.h>
#include <Urho3D/Audio/SoundSource3D.h>
#include <Urho3D/Audio/SoundListener.h>
#include <Urho3D/Audio/Audio.h>
#include <Urho3D/Graphics/ParticleEmitter.h>
#include <Urho3D/Graphics/ParticleEffect.h>
#include <Urho3D/Graphics/Terrain.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Physics/PhysicsEvents.h>
#include <Urho3D/Physics/Constraint.h>
#include "Additional/TailGenerator.h"

using namespace Urho3D;

/// \brief Calls SetModel on the given model and tries to load the model file and all texture files mentioned in a model_name+".txt".
/// model_name is supposed to have no file extension. Example: "Data/Models/Box", loads the model "Data/Models/Box.mdl".
/// It's a template to support all model classes like AnimatedModel and StaticModel.
template<typename T>
void set_model(T* model, Urho3D::ResourceCache* cache, std::string model_name)
{
	std::string filename_model = model_name;
	model->SetModel(cache->GetResource<Urho3D::Model>(Urho3D::String(filename_model.append(".mdl").c_str())));
	std::string filename_txt = model_name;
	filename_txt.append(".txt");
	std::ifstream file(filename_txt.c_str());
	std::string line;
	if (file.is_open())
	for (int i = 0; getline(file, line); i++)
		model->SetMaterial(i, cache->GetResource<Urho3D::Material>(Urho3D::String(line.c_str())));
}

/// SampleApplication main class mainly used for setup. The control is then given to the game states (starting with gs_main_menu).
class SampleApplication : public Application
{
public:
	enum UMask : const unsigned {
		none = 0,
		car = (1 << 0),
		ground = (1 << 1),
		all = Urho3D::M_MAX_UNSIGNED
	};

	PhysicsWorld* physicsWorld;

	SharedPtr<Scene> scene_;
	DebugRenderer* debugRenderer;

	Node* cameraNode_;
	SharedPtr<Urho3D::Node>
		node_car,
		node_car_model,
		node_car_model_glass,
		node_car_model_trail1,
		node_car_model_trail2;

	TailGenerator* turbo_trail1, *turbo_trail2;

	float simulated_engine_force = 0;

	SoundSource3D
		*sound_turbo_release_source,
		*sound_turbo_hold_source,
		*sound_engine_source;
	Sound
		*sound_turbo_release_1,
		*sound_turbo_release_2,
		*sound_turbo_release_3,
		*sound_turbo_hold,
		*sound_engine;

	Urho3D::Text* window_text;
	SharedPtr<Urho3D::Window> window;
	Urho3D::Camera* camera_;
	SharedPtr<Node> skyNode;
	SharedPtr<Node> node_torch;
	SharedPtr<Node> lightNode;
	Terrain* terrain;

	Vector3 rotatedLB, rotatedRB, rotatedLF, rotatedRF;

	float current_direction;
	Vector3 current_gravity;

	RigidBody* car_body;

	double running_time = 0.0;

	float
		car_acceleration = 3000,
		car_turbo_acceleration = 6000,
		car_brake = 4000,
		car_friction = 16000,
		car_steer = 1000,
		car_skid_power = 40,
		car_jump_power = 10;

	bool car_turbo = false;
	float car_speed = false;

	float floating_height = 5.0, floating_height_min = 0.3, floating_height_max = 1.2, floating_height_traction = 5.0;

	float
		amountOfForceLB = 0,
		amountOfForceRB = 0,
		amountOfForceLF = 0,
		amountOfForceRF = 0;

	float half_width = 1.2;
	float half_depth = 1.5;
	float half_height = 0.4;

	bool
		debug_car_raycast = true,
		debug_car_raycast_normal = false,
		debug_car_raycast_force = true,
		debug_gravity = true;


	bool
		input_engine = false,
		input_turbo = false,
		input_brake = false,
		input_jump = false;

	float input_direction = 0.0;
	bool on_ground = false, front_ground = false, rear_ground = false;

	float prevDist[4];
	PhysicsRaycastResult* prrlb, *prrrb, *prrlf, *prrrf, *prrm;

	SampleApplication(Context * context) : Application(context) {}

	virtual void Setup()
	{
		engineParameters_["FullScreen"] = false;
		engineParameters_["WindowWidth"] = 1280;
		engineParameters_["WindowHeight"] = 720;
		engineParameters_["WindowResizable"] = true;
		//engineParameters_["Multisample"]=16;
	}

	virtual void Start()
	{
		ResourceCache* cache = GetSubsystem<ResourceCache>();
		GetSubsystem<UI>()->GetRoot()->SetDefaultStyle(cache->GetResource<XMLFile>("UI/DefaultStyle.xml"));

		Engine* engine = GetSubsystem<Engine>();
		engine->SetMaxFps(1000);

		current_gravity = Vector3(0, 0, 0);

		scene_ = new Scene(context_);
		scene_->CreateComponent<Octree>();
		debugRenderer = scene_->CreateComponent<DebugRenderer>();

		TailGenerator::RegisterObject(context_);

		physicsWorld = scene_->CreateComponent<PhysicsWorld>();

		cameraNode_ = scene_->CreateChild("Camera");
		camera_ = cameraNode_->CreateComponent<Camera>();
		camera_->SetFarClip(600);
		camera_->SetNearClip(0.1);
		camera_->SetFov(75);

		SoundListener* listener = cameraNode_->CreateComponent<SoundListener>();
		GetSubsystem<Audio>()->SetListener(listener);
		GetSubsystem<Audio>()->SetMasterGain(SOUND_MUSIC, 0.3);

		Renderer* renderer = GetSubsystem<Renderer>();
		SharedPtr<Viewport> viewport(new Viewport(context_, scene_, cameraNode_->GetComponent<Camera>()));
		renderer->SetViewport(0, viewport);
		renderer->SetShadowMapSize(1024);

		RenderPath* effectRenderPath = viewport->GetRenderPath();
		effectRenderPath->Append(cache->GetResource<XMLFile>("PostProcess/AutoExposure.xml"));
		//effectRenderPath->Append(cache->GetResource<XMLFile>("PostProcess/BloomHDR.xml"));
		effectRenderPath->Append(cache->GetResource<XMLFile>("PostProcess/BloomHDR_stronger.xml"));
		effectRenderPath->Append(cache->GetResource<XMLFile>("PostProcess/FXAA2.xml"));

		Node* zoneNode = scene_->CreateChild("Zone");
		Zone* zone = zoneNode->CreateComponent<Zone>();
		zone->SetBoundingBox(BoundingBox(-50000.0f, 50000.0f));
		zone->SetFogStart(500.0f);
		zone->SetFogEnd(600.0f);
		zone->SetFogColor(Color(0.1, 0.2, 0.6));
		zone->SetAmbientColor(Color(1, 1, 1));

		SubscribeToEvent(E_KEYDOWN, URHO3D_HANDLER(SampleApplication, HandleKeyDown));
		SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(SampleApplication, HandleUpdate));
		SubscribeToEvent(E_POSTRENDERUPDATE, URHO3D_HANDLER(SampleApplication, HandlePostRenderUpdate));
		SubscribeToEvent(E_JOYSTICKCONNECTED, URHO3D_HANDLER(SampleApplication, HandleJoystickConnect));
		SubscribeToEvent(E_PHYSICSPRESTEP, URHO3D_HANDLER(SampleApplication, HandlePhysicsUpdate));


		cameraNode_->SetDirection(Vector3::FORWARD);

		// create a transparent window with some text to display things like help and FPS
		{
			window = new Window(context_);
			GetSubsystem<UI>()->GetRoot()->AddChild(window);
			window->SetStyle("Window");
			window->SetSize(600, 200);
			window->SetColor(Color(.0, .15, .3, .5));
			window->SetAlignment(HA_LEFT, VA_TOP);

			window_text = new Text(context_);
			window_text->SetFont(cache->GetResource<Font>("Fonts/Anonymous Pro.ttf"), 14);
			window_text->SetColor(Color(.8, .85, .9));
			window_text->SetAlignment(HA_LEFT, VA_TOP);
			window->AddChild(window_text);
		}

		// skybox
		{
			skyNode = scene_->CreateChild("Sky");
			skyNode->SetScale(1500.0f);
			Skybox* skybox = skyNode->CreateComponent<Skybox>();
			skybox->SetModel(cache->GetResource<Model>("Models/Box.mdl"));
			skybox->SetMaterial(cache->GetResource<Material>("Materials/Skybox.xml"));
		}

		// Car
		{
			node_car = scene_->CreateChild("Car");
			node_car->SetPosition(Vector3(-4, 1.6, 6));
			//node_car->Scale(0.5f);

			car_body = node_car->CreateComponent<RigidBody>();
			car_body->SetMass(1);
			car_body->SetAngularDamping(0.8);
			car_body->SetLinearDamping(0.2);
			//car_body->SetCcdRadius(100);
			//car_body->SetCcdMotionThreshold(20);
			//car_body->SetFriction(0);
			//car_body->SetRollingFriction(0);
			car_body->SetCollisionLayer(car);
			car_body->SetUseGravity(false);
			//car_body->SetCollisionMask(car);

			CollisionShape * car_shape = node_car->CreateComponent<CollisionShape>();
			car_shape->SetBox(Vector3(half_width * 2, half_height * 2, half_depth * 2));
			car_shape->SetPosition(Vector3::UP*half_height);

			node_car_model = node_car->CreateChild("Model");
			node_car_model_glass = node_car_model->CreateChild("Glass");

			node_car_model_trail1 = node_car_model->CreateChild("Trail1");
			node_car_model_trail1->SetPosition(Vector3(half_width - 0.6, half_height * 2 + 0.4, -half_depth + 0.1));

			node_car_model_trail2 = node_car_model->CreateChild("Trail2");
			node_car_model_trail2->SetPosition(Vector3(-half_width + 0.6, half_height * 2 + 0.4, -half_depth + 0.1));

			sound_turbo_hold = cache->GetResource<Sound>("Sounds/turbo_hold.wav");
			sound_turbo_hold_source = node_car_model->CreateComponent<SoundSource3D>();
			sound_turbo_hold_source->SetNearDistance(1);
			sound_turbo_hold_source->SetFarDistance(50);
			sound_turbo_hold_source->SetSoundType(SOUND_EFFECT);

			sound_engine = cache->GetResource<Sound>("Sounds/engine.wav");
			sound_engine->SetLooped(true);
			sound_engine_source = node_car_model->CreateComponent<SoundSource3D>();
			sound_engine_source->SetNearDistance(1);
			sound_engine_source->SetFarDistance(50);
			sound_engine_source->SetSoundType(SOUND_EFFECT);
			sound_engine_source->Play(sound_engine);

			sound_turbo_release_1 = cache->GetResource<Sound>("Sounds/turbo_release_1.wav");
			sound_turbo_release_2 = cache->GetResource<Sound>("Sounds/turbo_release_2.wav");
			sound_turbo_release_3 = cache->GetResource<Sound>("Sounds/turbo_release_3.wav");
			sound_turbo_release_source = node_car_model->CreateComponent<SoundSource3D>();
			sound_turbo_release_source->SetNearDistance(1);
			sound_turbo_release_source->SetFarDistance(50);
			sound_turbo_release_source->SetSoundType(SOUND_EFFECT);
			//sound_turbo_release_source->Play(sound_turbo_release);

			turbo_trail1 = node_car_model_trail1->CreateComponent<TailGenerator>();
			turbo_trail1->SetDrawVertical(false);
			turbo_trail1->SetTailLength(0.1f); // set segment length
			turbo_trail1->SetNumTails(0);     // set num of segments
			turbo_trail1->SetWidthScale(0.5f); // side scale
			turbo_trail1->SetColorForHead(Color(0.2f, 0.5f, 10.0f, 1.0f));
			turbo_trail1->SetColorForTip(Color(1.0f, 1.0f, 0.0f, 0.0f));

			turbo_trail2 = node_car_model_trail2->CreateComponent<TailGenerator>();
			turbo_trail2->SetDrawVertical(false);
			turbo_trail2->SetTailLength(0.1f); // set segment length
			turbo_trail2->SetNumTails(0);     // set num of segments
			turbo_trail2->SetWidthScale(0.5f); // side scale
			turbo_trail2->SetColorForHead(Color(0.2f, 0.5f, 10.0f, 1.0f));
			turbo_trail2->SetColorForTip(Color(1.0f, 1.0f, 0.0f, 0.0f));

			//node_car_model->SetDirection(Vector3::RIGHT);

			StaticModel* carFrame = node_car_model->CreateComponent<StaticModel>();
			carFrame->SetModel(cache->GetResource<Model>("Models/Subamru/Hull.mdl"));
			carFrame->SetMaterial(cache->GetResource<Material>("Materials/BrightRed.xml"));
			carFrame->SetCastShadows(true);
			//carFrame->SetEnabled(false);

			StaticModel* carPipes = node_car_model->CreateComponent<StaticModel>();
			carPipes->SetModel(cache->GetResource<Model>("Models/Subamru/Pipeline.mdl"));
			carPipes->SetMaterial(cache->GetResource<Material>("Materials/Black_RedSpec.xml"));
			carPipes->SetCastShadows(true);

			StaticModel* carGlass = node_car_model->CreateComponent<StaticModel>();
			carGlass->SetModel(cache->GetResource<Model>("Models/Subamru/Windows.mdl"));
			carGlass->SetMaterial(cache->GetResource<Material>("Materials/Glass.xml"));
			carGlass->SetCastShadows(true);
		}

		// a torch with a light, sound and particle effects
		{
			node_torch = scene_->CreateChild("Torch");
			Vector3 pos(Vector3(3, -0.3, 6));
			node_torch->SetPosition(pos);

			StaticModel* boxObject = node_torch->CreateComponent<StaticModel>();
			set_model(boxObject, cache, "Data/Models/torch");
			boxObject->SetCastShadows(true);
			boxObject->SetOccludee(true);
			boxObject->SetShadowDistance(200);
			boxObject->SetDrawDistance(200);

			Node* lightNode = node_torch->CreateChild();
			lightNode->Translate(Vector3(0, 2, 0));
			Light* light = lightNode->CreateComponent<Light>();
			light->SetLightType(LIGHT_POINT);
			light->SetRange(50);
			light->SetBrightness(1.2);
			light->SetColor(Color(1.0, 1.0, 1.0, 1.0));
			light->SetCastShadows(true);
			light->SetShadowDistance(200);
			light->SetDrawDistance(200);

			Node* n_particle = node_torch->CreateChild();
			n_particle->Translate(Vector3(0, 1.6, 0));
			ParticleEmitter* emitter = n_particle->CreateComponent<ParticleEmitter>();
			emitter->SetEffect(cache->GetResource<ParticleEffect>("Particle/torch_fire.xml"));
			emitter = n_particle->CreateComponent<ParticleEmitter>();
			emitter->SetEffect(cache->GetResource<ParticleEffect>("Particle/torch_smoke.xml"));

			Sound* sound_torch = cache->GetResource<Sound>("Sounds/torch.ogg");
			sound_torch->SetLooped(true);
			SoundSource3D* sound_torch_source = n_particle->CreateComponent<SoundSource3D>();
			sound_torch_source->SetNearDistance(1);
			sound_torch_source->SetFarDistance(50);
			sound_torch_source->SetSoundType(SOUND_EFFECT);
			sound_torch_source->Play(sound_torch);
		}

		// sun
		{
			lightNode = scene_->CreateChild("Light");
			Light* light = lightNode->CreateComponent<Light>();
			light->SetLightType(LIGHT_DIRECTIONAL);
			light->SetCastShadows(true);
			light->SetShadowBias(BiasParameters(0.00025f, 0.7f));
			light->SetShadowCascade(CascadeParameters(4.0f, 16.0f, 64.0f, 128.0f, 0.8f));
			light->SetColor(Color(1, 1, 1, 1));
			lightNode->SetDirection(Vector3::FORWARD);
			lightNode->Yaw(-150);   // horizontal
			lightNode->Pitch(30);   // vertical
			lightNode->Translate(Vector3(0, 0, -20000));

			BillboardSet* billboardObject = lightNode->CreateComponent<BillboardSet>();
			billboardObject->SetNumBillboards(1);
			billboardObject->SetMaterial(cache->GetResource<Material>("Materials/sun.xml"));
			billboardObject->SetSorted(true);
			Billboard* bb = billboardObject->GetBillboard(0);
			bb->size_ = Vector2(10000, 10000);
			bb->rotation_ = Random()*360.0f;
			bb->enabled_ = true;
			billboardObject->Commit();
		}
		// terrain
		/*
	 {
	 Node* terrainNode = scene_->CreateChild("Terrain");
	 terrainNode->SetPosition(Vector3(3.0f, -0.4f));
	 terrain = terrainNode->CreateComponent<Terrain>();
	 terrain->SetPatchSize(128);
	 terrain->SetSpacing(Vector3(2, 0.5, 2));
	 terrain->SetSmoothing(true);
	 terrain->SetHeightMap(cache->GetResource<Image>("Textures/HeightMap.png"));
	 terrain->SetMaterial(cache->GetResource<Material>("Materials/Terrain.xml"));
	 terrain->SetCastShadows(true);
	 terrain->SetOccluder(true);
	 RigidBody * terrainBody = terrainNode->CreateComponent<RigidBody>();
	 terrainBody->SetCollisionLayer(ground);
	 CollisionShape* terrain_shape = terrainNode->CreateComponent<CollisionShape>();
	 terrain_shape->SetTerrain();
	 }
	 */

		{
			Node* terrainNode = scene_->CreateChild("Terrain");
			terrainNode->SetPosition(Vector3(2, -10));
			terrainNode->SetScale(4);
			StaticModel* planeModel = terrainNode->CreateComponent<StaticModel>();
			set_model(planeModel, cache, "Models/bowltrack");
			RigidBody * terrainBody = terrainNode->CreateComponent<RigidBody>();
			terrainBody->SetCollisionLayer(ground);
			CollisionShape* terrain_shape = terrainNode->CreateComponent<CollisionShape>();
			//terrain_shape->SetBox(Vector3(100, 1, 100));
			planeModel->SetMaterial(cache->GetResource<Material>("Materials/StoneTiled.xml"));
			terrain_shape->SetTriangleMesh(GetSubsystem<ResourceCache>()->GetResource<Model>("Models/bowltrack.mdl"));
		}


		prrlb = new PhysicsRaycastResult();
		prrrb = new PhysicsRaycastResult();
		prrlf = new PhysicsRaycastResult();
		prrrf = new PhysicsRaycastResult();
		prrm = new PhysicsRaycastResult();

	}

	virtual void Stop()
	{
	}

	void HandleUpdate(StringHash eventType, VariantMap& eventData)
	{
		float timeStep = eventData[Update::P_TIMESTEP].GetFloat();

		running_time += timeStep;

		JoystickState* js = GetSubsystem<Input>()->GetJoystick(0);

		front_ground = prrlf->body_ || prrrf->body_;
		rear_ground = prrlb->body_ || prrrb->body_;
		on_ground = prrlb->body_ || prrrb->body_ || prrlf->body_ || prrrf->body_;

		if (prrlb->body_ && prrrb->body_ && prrlf->body_ && prrrf->body_)
			floating_height = floating_height_traction;
		else
			floating_height = floating_height_max;

		car_speed = car_body->GetLinearVelocity().ProjectOntoAxis(node_car->GetDirection());

		input_engine = js->GetButtonDown(CONTROLLER_BUTTON_A);
		input_turbo = js->GetButtonDown(CONTROLLER_BUTTON_X);
		input_brake = js->GetButtonDown(CONTROLLER_BUTTON_B);
		input_jump = js->GetButtonPress(CONTROLLER_BUTTON_Y);

		input_direction = js->GetAxisPosition(CONTROLLER_AXIS_LEFTX);

		float vectorRotateCos = Cos(-node_car->GetRotation().EulerAngles().y_);
		float vectorRotateSin = Sin(-node_car->GetRotation().EulerAngles().y_);



		if (debug_car_raycast_normal){
			debugRenderer->AddLine(prrrf->position_, prrrf->position_ + prrrf->normal_, Color(1, 0, 0));
			debugRenderer->AddLine(prrrb->position_, prrrb->position_ + prrrb->normal_, Color(1, 0, 0));
			debugRenderer->AddLine(prrlf->position_, prrlf->position_ + prrlf->normal_, Color(1, 0, 0));
			debugRenderer->AddLine(prrlb->position_, prrlb->position_ + prrlb->normal_, Color(1, 0, 0));
		}
		if (debug_car_raycast && on_ground){
			debugRenderer->AddLine(node_car->LocalToWorld(Vector3(+half_width, 0, +half_depth)), prrrf->position_, Color(1, 1, 1, 1));
			debugRenderer->AddLine(node_car->LocalToWorld(Vector3(-half_width, 0, +half_depth)), prrrb->position_, Color(1, 1, 1, 1));
			debugRenderer->AddLine(node_car->LocalToWorld(Vector3(+half_width, 0, -half_depth)), prrlf->position_, Color(1, 1, 1, 1));
			debugRenderer->AddLine(node_car->LocalToWorld(Vector3(-half_width, 0, -half_depth)), prrlb->position_, Color(1, 1, 1, 1));
		}

		if (debug_gravity){
			debugRenderer->AddLine(node_car->GetPosition(), node_car->GetPosition() + current_gravity, Color(0.2, 0.3, 1, 1), false);
		}

		if (debug_car_raycast_force){
			debugRenderer->AddLine(node_car->LocalToWorld(Vector3(-half_width, 0, -half_depth)), node_car->LocalToWorld(Vector3(-half_width, 0, -half_depth)) + node_car->GetUp()*amountOfForceLB / 4, Color(0.8, 0.2, 0.1, 1));
			debugRenderer->AddLine(node_car->LocalToWorld(Vector3(-half_width, 0, +half_depth)), node_car->LocalToWorld(Vector3(-half_width, 0, +half_depth)) + node_car->GetUp()*amountOfForceRB / 4, Color(0.8, 0.2, 0.1, 1));
			debugRenderer->AddLine(node_car->LocalToWorld(Vector3(+half_width, 0, -half_depth)), node_car->LocalToWorld(Vector3(+half_width, 0, -half_depth)) + node_car->GetUp()*amountOfForceLF / 4, Color(0.8, 0.2, 0.1, 1));
			debugRenderer->AddLine(node_car->LocalToWorld(Vector3(+half_width, 0, +half_depth)), node_car->LocalToWorld(Vector3(+half_width, 0, +half_depth)) + node_car->GetUp()*amountOfForceRF / 4, Color(0.8, 0.2, 0.1, 1));
		}
		//debugRenderer->AddSphere(Sphere(node_car->LocalToWorld(Vector3(0, -half_height, -half_depth / 2)), 0.1), Color::WHITE, false);


		sound_engine_source->SetFrequency(12000 + 800 * simulated_engine_force);
		sound_engine_source->SetGain(Min((simulated_engine_force - 2) / 5, 3));


		if (input_engine){
			if (car_turbo)
				simulated_engine_force = Lerp(simulated_engine_force, car_turbo_acceleration * 5 * timeStep, 0.001);
			else
				simulated_engine_force = Lerp(simulated_engine_force, car_acceleration * 5 * timeStep, 0.001);
		}
		else{
			if (input_brake)
				simulated_engine_force = Lerp(simulated_engine_force, 0.0f, 0.01);
			else
				simulated_engine_force = Lerp(simulated_engine_force, 0.0f, 0.001);


		}

		if (on_ground){

			if (js->GetButtonDown(CONTROLLER_BUTTON_DPAD_LEFT)){
				car_body->ApplyTorque(node_car->GetUp()*-car_steer * timeStep);
				car_body->ApplyTorque(node_car->GetDirection()*-2000 * timeStep);
			}
			if (js->GetButtonDown(CONTROLLER_BUTTON_DPAD_RIGHT)){
				car_body->ApplyTorque(node_car->GetUp()*car_steer * timeStep);
				car_body->ApplyTorque(node_car->GetDirection() * 2000 * timeStep);
			}

			//car_body->ApplyTorque(node_car->GetUp() * 500 * timeStep* input_direction);

			if (js->GetButtonDown(CONTROLLER_BUTTON_DPAD_DOWN))
				car_body->ApplyForce(node_car->GetUp() * 5,
				node_car->LocalToWorld(Vector3(0, -half_height, +half_depth)) - node_car->GetPosition()
				);

			if (input_engine){
				if (car_turbo){
					if (car_speed < 70)
						car_body->ApplyForce(node_car->GetDirection()* car_turbo_acceleration*timeStep);
					car_body->ApplyTorque(node_car->GetRight() * -3000 * timeStep);
					//car_body->ApplyForce(node_car->GetUp() * -4000 * timeStep);
				}
				else{
					car_body->ApplyForce(node_car->GetDirection()* car_acceleration*timeStep);
					car_body->ApplyTorque(node_car->GetRight()*-2000 * timeStep);
					//car_body->ApplyForce(node_car->GetUp() * -2000 * timeStep);
				}
			}
			else{
				if (car_turbo){	  // Shake car
					car_body->ApplyForce(node_car->GetUp()*-500 * timeStep,
						Vector3::RIGHT*Sin(running_time * 10 / timeStep) * 5 + Vector3::FORWARD*Cos(running_time * 10 / timeStep) * 5);
				}
			}


			if (input_brake && car_speed > 0){
				car_body->ApplyForce(-node_car->GetDirection()* car_brake*timeStep);
				car_body->ApplyTorque(node_car->GetRight() * 3000 * timeStep);
			}

			if (input_turbo){
				if (car_turbo == false){
					car_turbo = true;
					turbo_trail1->SetNumTails(10);     // set num of segments
					turbo_trail2->SetNumTails(10);     // set num of segments
					sound_turbo_hold_source->Play(sound_turbo_hold);
				}
			}
			else {
				if (car_turbo == true){
					car_turbo = false;
					turbo_trail1->SetNumTails(0);     // set num of segments
					turbo_trail2->SetNumTails(0);     // set num of segments
					sound_turbo_hold_source->Stop();
					switch (rand() % 3)
					{
					case 0:
						sound_turbo_release_source->Play(sound_turbo_release_1);
						break;
					case 1:
						sound_turbo_release_source->Play(sound_turbo_release_2);
						break;
					case 2:
						sound_turbo_release_source->Play(sound_turbo_release_3);
						break;
					}
				}
			}
			if (input_jump)
				car_body->ApplyImpulse(Vector3::UP * car_jump_power);

			if (js->GetButtonPress(CONTROLLER_BUTTON_LEFTSHOULDER))
				car_body->ApplyImpulse(node_car->GetRight()*Vector3(-1, -1, -1) * car_skid_power);

			if (js->GetButtonPress(CONTROLLER_BUTTON_RIGHTSHOULDER))
				car_body->ApplyImpulse(node_car->GetRight()*Vector3(1, 1, 1) * car_skid_power);

			if (car_body->GetLinearVelocity().ProjectOntoAxis(node_car->GetRight()) > 0.3)
				car_body->ApplyForce(node_car->GetRight()*Vector3(-1, -1, -1)* car_friction*timeStep);

			if (car_body->GetLinearVelocity().ProjectOntoAxis(-node_car->GetRight()) > 0.3)
				car_body->ApplyForce(node_car->GetRight()*Vector3(1, 1, 1)* car_friction*timeStep);

		}
		else{
			if (input_jump)
				car_body->ApplyImpulse(Vector3::DOWN * car_jump_power);
		}
		//Keep the car up

		car_body->ApplyTorque(-node_car->GetUp().CrossProduct(current_gravity.Normalized()) * 1000 * timeStep);

		// Movement speed as world units per second
		float MOVE_SPEED = 10.0f;
		// Mouse sensitivity as degrees per pixel
		const float MOUSE_SENSITIVITY = 0.1f;

		// camera movement
		Input* input = GetSubsystem<Input>();

		//rotatedLB = Vector3((-half_width)*vectorRotateCos - (-half_depth)*vectorRotateSin, 0, (-half_width)*vectorRotateSin + (-half_depth)*vectorRotateCos);
		//rotatedRB = Vector3((-half_width)*vectorRotateCos - (+half_depth)*vectorRotateSin, 0, (-half_width)*vectorRotateSin + (+half_depth)*vectorRotateCos);
		//rotatedLF = Vector3((+half_width)*vectorRotateCos - (-half_depth)*vectorRotateSin, 0, (+half_width)*vectorRotateSin + (-half_depth)*vectorRotateCos);
		//rotatedRF = Vector3((+half_width)*vectorRotateCos - (+half_depth)*vectorRotateSin, 0, (+half_width)*vectorRotateSin + (+half_depth)*vectorRotateCos);

		Ray rayLeftBack(node_car->LocalToWorld(Vector3(-half_width, 0, -half_depth)), -node_car->GetUp());
		Ray rayRightBack(node_car->LocalToWorld(Vector3(-half_width, 0, +half_depth)), -node_car->GetUp());
		Ray rayLeftFront(node_car->LocalToWorld(Vector3(+half_width, 0, -half_depth)), -node_car->GetUp());
		Ray rayRightFront(node_car->LocalToWorld(Vector3(+half_width, 0, +half_depth)), -node_car->GetUp());

		Ray rayMiddleF(node_car->GetPosition(), -node_car->GetUp() + node_car->GetDirection());
		Ray rayMiddleM(node_car->GetPosition(), -node_car->GetUp());
		Ray rayMiddleB(node_car->GetPosition(), -node_car->GetUp() - node_car->GetDirection());

		physicsWorld->RaycastSingle(*prrlb, rayLeftBack, floating_height, ground);
		physicsWorld->RaycastSingle(*prrrb, rayRightBack, floating_height, ground);
		physicsWorld->RaycastSingle(*prrlf, rayLeftFront, floating_height, ground);
		physicsWorld->RaycastSingle(*prrrf, rayRightFront, floating_height, ground);

		//physicsWorld->RaycastSingle(*prrm, rayMiddle, floating_height, ground);
		physicsWorld->RaycastSingle(*prrm, rayMiddleM, 30, ground);

		if (prrm->body_)
			current_gravity = -prrm->normal_;
		else{
			physicsWorld->RaycastSingle(*prrm, rayMiddleF, 30, ground);

			if (prrm->body_)
				current_gravity = -prrm->normal_;
			else{

				physicsWorld->RaycastSingle(*prrm, rayMiddleB, 30, ground);

				if (prrm->body_)
					current_gravity = -prrm->normal_;
			}
		}

		//cameraNode_->SetPosition(node_car->GetPosition() + Vector3(0, 5, -10));
		cameraNode_->SetPosition(Lerp(cameraNode_->GetPosition(), node_car->LocalToWorld(Vector3(0, 3, -5))*Vector3(1, 1, 1), 0.1));
		//cameraNode_->LookAt(node_car->LocalToWorld(Vector3::UP * 3));
		cameraNode_->SetRotation(cameraNode_->GetRotation().Slerp(node_car->GetRotation(), 0.06));
		camera_->SetFov(car_speed*0.5 + 70);

		std::string str = "WASD, mouse and shift to move. T to toggle fill mode,\nG to toggle GUI, Tab to toggle mouse mode, Esc to quit.\n Velocity: ";
		{
			std::ostringstream ss;
			ss << car_speed;
			std::string s(ss.str());
			str.append(s.substr(0));
		}
		str.append("");
		String s(str.c_str(), str.size());
		window_text->SetText(s);

		if (js->GetButtonDown(CONTROLLER_BUTTON_START)){
			car_body->ResetForces();
			car_body->SetLinearVelocity(Vector3::ZERO);
			car_body->SetAngularVelocity(Vector3::ZERO);
			node_car->SetPosition(Vector3(-4, 0, 0));
			node_car->SetRotation(Quaternion());
		}

	}

	void HandlePhysicsUpdate(StringHash eventType, VariantMap& eventData){

		if (prrlb->body_){
			amountOfForceLB = calculateCarRaycastForce(0);
			prevDist[0] = prrlb->distance_;
			car_body->ApplyForce(node_car->GetUp()*amountOfForceLB,
				node_car->LocalToWorld(Vector3(-half_width, 0, -half_depth)) - node_car->GetPosition()
				);
		}
		else{
			prevDist[0] = floating_height;
		}

		if (prrrb->body_){
			amountOfForceRB = calculateCarRaycastForce(1);
			prevDist[1] = prrrb->distance_;
			car_body->ApplyForce(node_car->GetUp()*amountOfForceRB,
				node_car->LocalToWorld(Vector3(-half_width, 0, +half_depth)) - node_car->GetPosition()
				);
		}
		else{
			prevDist[1] = floating_height;
		}

		if (prrlf->body_){
			amountOfForceLF = calculateCarRaycastForce(2);
			prevDist[2] = prrlf->distance_;
			car_body->ApplyForce(node_car->GetUp()*amountOfForceLF,
				node_car->LocalToWorld(Vector3(+half_width, 0, -half_depth)) - node_car->GetPosition()
				);
		}
		else{
			prevDist[2] = floating_height;
		}

		if (prrrf->body_){
			amountOfForceRF = calculateCarRaycastForce(3);
			prevDist[3] = prrrf->distance_;
			car_body->ApplyForce(node_car->GetUp()*amountOfForceRF,
				node_car->LocalToWorld(Vector3(+half_width, 0, +half_depth)) - node_car->GetPosition()
				);
		}
		else{
			prevDist[3] = floating_height;
		}

		car_body->ApplyForce(current_gravity * 20);

	}

	void HandleJoystickConnect(StringHash eventType, VariantMap& eventData)
	{
		PrintLine("Joystick connected.");
	}

	void HandleKeyDown(StringHash eventType, VariantMap& eventData)
	{
		using namespace KeyDown;
		int key = eventData[P_KEY].GetInt();


		if (key == KEY_TAB)
		{
			GetSubsystem<Input>()->SetMouseVisible(!GetSubsystem<Input>()->IsMouseVisible());
			GetSubsystem<Input>()->SetMouseGrabbed(!GetSubsystem<Input>()->IsMouseGrabbed());
		}
		else if (key == KEY_ESCAPE)
			engine_->Exit();
		else if (key == KEY_G)
			window->SetVisible(!window->IsVisible());
		else if (key == KEY_T)
			camera_->SetFillMode(camera_->GetFillMode() == FILL_WIREFRAME ? FILL_SOLID : FILL_WIREFRAME);
	}

	void HandlePostRenderUpdate(StringHash eventType, VariantMap& eventData)
	{
		//scene_->GetComponent<PhysicsWorld>()->DrawDebugGeometry(true);
	}

	float calculateCarRaycastForce(int wheel){


		float floating_force = 15;
		float floating_damping_bump = 180;
		float floating_damping_rebound = 80;

		float compression = 0, prev_comp = 0;;
		float res = 0;
		float dist = 0, prev_dist = 0;

		float spring_force = 0, damping_force = 0;

		//F_spring = L_comp x k_susp;
		//F_damp = (L_comp - L_comp_prev)*k_damp;
		switch (wheel)
		{
		case 0:
			dist = prrlb->distance_;
			prev_dist = prevDist[0];
			break;
		case 1:
			dist = prrrb->distance_;
			prev_dist = prevDist[1];
			break;
		case 2:
			dist = prrlf->distance_;
			prev_dist = prevDist[2];
			break;
		case 3:
			dist = prrrf->distance_;
			prev_dist = prevDist[3];
			break;
		default:
			break;
		}
		compression = (floating_height_max - dist);
		prev_comp = (floating_height_max - prev_dist);

		if (dist < floating_height_min){
			compression += (floating_height_min - dist) * 20;
		}

		if (prev_dist < floating_height_min){
			floating_damping_rebound *= 5;
			prev_comp += (floating_height_min - prev_dist) * 20;
		}

		spring_force = compression*floating_force;
		if (compression - prev_comp > 0)
			damping_force = ((floating_height_max - dist) - (floating_height_max - prev_dist))*floating_damping_bump;
		else
			damping_force = ((floating_height_max - dist) - (floating_height_max - prev_dist))*floating_damping_rebound;
		res = spring_force + damping_force;

		std::string str = "";
		{
			std::ostringstream ss;
			ss << spring_force;
			std::string s(ss.str());
			str.append(s.substr(0, 6));
		}
		String s(str.c_str(), str.size());
		//PrintLine(s);

		return res;
	}

	unsigned Layer(unsigned x) { return 1 << (x - 1); }

	Vector3 RotateVector3(Vector3 v, float a){
		return Vector3(v.x_*Cos(-a) - v.z_*Sin(-a), v.y_, v.x_*Sin(-a) + v.z_*Cos(-a));
	}


};

URHO3D_DEFINE_APPLICATION_MAIN(SampleApplication)
