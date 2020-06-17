using System;
using System.Linq;
using System.Collections.Generic;
using Godot;

public class Movement : KinematicBody
{
	[Export]
	public NodePath Camera;
	private OrbitCamera _camera;


	[Export]
	public bool useGodotContacts = false;

	// Via Marble Blast
	[Export] public float MaxRollVelocity = 15f;
	[Export] public float AngularAcceleration = 75f;
	[Export] public float BrakingAcceleration = 30f;
	[Export] public float AirAcceleration = 5f;
	[Export] public float Gravity = 20f;
	[Export] public float StaticFriction = 1.1f;
	[Export] public float KineticFriction = 0.7f;
	[Export] public float BounceKineticFriction = 0.2f;
	[Export] public float MaxDotSlide = 0.5f;
	[Export] public float JumpImpulse = 7.5f;
	[Export] public float MaxForceRadius = 50f;
	[Export] public float MinBounceVel = 0.1f;
	[Export] public float BounceRestitution = 0.5f;

	private float _remainingTime = 0.0f;

	// TODO: Clean up / rename the following
	private float CameraX => _camera.yaw;
	private float CameraY => _camera.pitch;
	private Vector3 Velocity;
	private Vector3 AngularVelocity;
	private float Radius=0;

	private Vector2 InputMovement {
		get {
			var joy = new Vector2(
				Input.GetJoyAxis(0, 0),
				Input.GetJoyAxis(0, 1)
			) + _fakeInput;

			if (joy.LengthSquared() > 0)
				return joy;

			var keyboard = new Vector2();
			if (Input.IsActionPressed("move_left"))  keyboard.x -= 1f;
			if (Input.IsActionPressed("move_right")) keyboard.x += 1f;
			if (Input.IsActionPressed("move_up"))    keyboard.y += 1f;
			if (Input.IsActionPressed("move_down"))  keyboard.y -= 1f;
			return keyboard;
		}
	}
	private Vector2 _fakeInput = Vector2.Zero;

	private bool Jump => Input.IsActionPressed("jump");

	public Vector3 GravityDir = Vector3.Down;

	private bool _bounceYet;
	private float _bounceSpeed;
	private Vector3 _bouncePos;
	private Vector3 _bounceNormal;
	private float _slipAmount;
	private float _contactTime;
	private float _rollVolume;

	private List<CollisionShape> _colTests;

	class MeshData
	{
		public int[] Triangles;
		public Vector3[] Vertices;
	}


	private KinematicBody _rigidBody;
	private CollisionShape _collider;
	private int _collisions;
	private float _lastJump;
	private Vector3 _lastNormal;

	struct CollisionInfo
	{
		public Vector3 Point;
		public Vector3 Normal;
		public Vector3 Velocity;
		public Node Collider;
		public float Friction;
		public float Restitution;
		public float Penetration;
	}

	public override void _Ready() {
		_camera = (OrbitCamera)GetNode<OrbitCamera>(Camera);
		_rigidBody = this;
		_collider = GetNode<CollisionShape>("CollisionShape");
		Radius = _collider.Scale.x/2;
	}
	
	/*
function rmtest() {
	$mp::mymarble.setcollisionradius(0.2);
	$mp::mymarble.setvelocity("0 0 0");
	$mp::mymarble.setangularvelocity("0 0 0");
	$mp::mymarble.setcamerayaw(0);
	$mp::mymarble.setcamerapitch(0.45);
	$MP::MyMarble.settransform("16 -28 -8.8 0 0 0 1");
	b();
	schedule(1000, 0, "eval", "moveforward(1);$firstf = getRealTime();");
	schedule(5000, 0, "eval", "cancel($b);");
	echo("t,px,py,pz,vx,vy,vz,ox,oy,oz");
}
function f() {
	if (getRealTime() $= $lastf)
		return;
	$lastf = getRealTime();
	echo(strReplace((getRealTime() - $firstf) SPC $MP::MyMarble.getPosition() SPC $MP::MyMarble.getVelocity() SPC $MP::MyMarble.getAngularVelocity(), " ", ","));
}
	 */

	public override void _PhysicsProcess(float delta) {
		float dt = delta;
		_remainingTime += dt;
		while (_remainingTime > 0.008f)
		{
			float loopTime = 0.008f;
			_advancePhysics(ref loopTime);
			_remainingTime -= loopTime;
		}
	}

	private static void PrintVector3(Vector3 p) {
		GD.Print("(" +
			p.x.ToString("0.####") + ","+
			p.y.ToString("0.####") + "," +
			p.z.ToString("0.####") + 
		")");
	}

	private Queue<Node> _recycleNodes = new Queue<Node>();

	void _advancePhysics(ref float dt)
	{
		List<CollisionInfo> contacts = new List<CollisionInfo>();
		Vector3 pos = Translation;
		Quat rot = new Quat(Rotation);
		Vector3 velocity = Velocity;
		Vector3 omega = AngularVelocity;

		if (useGodotContacts) {
			var root = GetTree().Root.GetNodeOrNull("Spatial/Collisions");
			if(root != null){
				foreach (Node item in root.GetChildren()) {
					_recycleNodes.Enqueue(item);
					root.RemoveChild(item);
				}
			}

			var collisionShapes = new List<CollisionShape>(5);
			while (MoveAndCollide(Vector3.Zero, true, true, true) is KinematicCollision collision) {
				var shape = (CollisionShape)collision.ColliderShape;
				collisionShapes.Add(shape);
				shape.Disabled = true;

				var penetration = Translation.DistanceTo(collision.Position) - Radius;
				var col = new CollisionInfo {
					Penetration = penetration,
					Restitution = 1,
					Friction = 1,
					Normal = collision.Normal,
					Point = collision.Position
				};
				/*if (contact.otherCollider.attachedRigidbody != null) {
					col.Collider = contact.otherCollider;
					col.Velocity = contact.otherCollider.attachedRigidbody.GetPointVelocity(contact.point);
				}*/
				contacts.Add(col);

				if(root!=null) { // draw debug lol
					Spatial spatial = null;
					if (_recycleNodes.Count > 0)
						spatial = (Spatial)_recycleNodes.Dequeue();
					if (spatial == null) {
						spatial = new Spatial();
						var meshInstance = new MeshInstance();
						meshInstance.Mesh = new SphereMesh();
						meshInstance.Scale = new Vector3(0.1f,0.1f,0.1f);
						spatial.AddChild(meshInstance);
					}
					spatial.Translation = Translation + collision.Normal * 0.3f;
					root.AddChild(spatial);
				}
			}
			foreach (var item in collisionShapes)
				item.Disabled = false;

		}

		_updateMove(ref dt, ref velocity, ref omega, contacts);
		// velocity += _gravityDir * _gravity * dt;

		_updateIntegration(dt, ref pos, ref rot, velocity, omega);

		if (useGodotContacts) {
			Translation = pos;
			Rotation = rot.GetEuler();
		}
		else {
			Translation = pos;
			Rotation = rot.GetEuler();
		}
		Velocity = velocity;
		AngularVelocity = omega;
	}

	protected virtual void _updateIntegration(float dt, ref Vector3 pos, ref Quat rot, Vector3 vel, Vector3 avel)
	{
		pos += vel * dt;
		Vector3 vector3 = avel;
		float num1 = vector3.Length();
		if (num1 <= 0.0000001)
			return;
		Quat quaternion = new Quat(vector3 * (1f / num1), dt * num1);
		quaternion = quaternion.Normalized();
		rot = quaternion * rot;
		rot = rot.Normalized();
	}

	private void _updateMove(
		ref float dt,
		ref Vector3 velocity,
		ref Vector3 angVelocity,
		List<CollisionInfo> contacts)
	{
		bool isMoving = _computeMoveForces(angVelocity, out var torque, out var targetAngVel);
		_velocityCancel(contacts, ref velocity, ref angVelocity, !isMoving, false);
		Vector3 externalForces = _getExternalForces(dt, contacts);
		_applyContactForces(dt, contacts, !isMoving, torque, targetAngVel, ref velocity, ref angVelocity, ref externalForces, out var angAccel);
		velocity += externalForces * dt;
		angVelocity += angAccel * dt;
		_velocityCancel(contacts, ref velocity, ref angVelocity, !isMoving, true);
		float contactTime = dt;
		// testMove(ref contactTime, ...)
		if (dt * 0.99 > contactTime)
		{
			velocity -= externalForces * (dt - contactTime);
			angVelocity -= angAccel * (dt - contactTime);
			dt = contactTime;
		}
		
		if (contacts.Count != 0)
			_contactTime += dt;
	}

	private bool _computeMoveForces(Vector3 angVelocity, out Vector3 torque, out Vector3 targetAngVel)
	{
		torque = Vector3.Zero;
		targetAngVel = Vector3.Zero;
		Vector3 relGravity = -GravityDir * Radius;
		Vector3 topVelocity = angVelocity.Cross(relGravity);
		_getMarbleAxis(out var sideDir, out var motionDir, out Vector3 _);
		float topY = topVelocity.Dot(motionDir);
		float topX = topVelocity.Dot(sideDir);
		Vector2 move = InputMovement;
		// move = move.Normalized();
		float moveY = MaxRollVelocity * move.y;
		float moveX = MaxRollVelocity * move.x;
		if (Math.Abs(moveY) < 0.001f && Math.Abs(moveX) < 0.001f)
			return false;
		if (topY > moveY && moveY > 0.0)
			moveY = topY;
		else if (topY < moveY && moveY < 0.0)
			moveY = topY;
		if (topX > moveX && moveX > 0.0)
			moveX = topX;
		else if (topX < moveX && moveX < 0.0)
			moveX = topX;
		targetAngVel = relGravity.Cross(moveY * motionDir + moveX * sideDir) / relGravity.LengthSquared();
		torque = targetAngVel - angVelocity;
		float targetAngAccel = torque.Length();
		if (targetAngAccel > AngularAcceleration)
		{
			torque *= AngularAcceleration / targetAngAccel;
		}

		return true;
	}

	private void _getMarbleAxis(out Vector3 sideDir, out Vector3 motionDir, out Vector3 upDir)
	{
        var m = new Quat(new Vector3(CameraY, 0, 0)) * new Quat(new Vector3(0, CameraX, 0));
		upDir = -GravityDir;
        var forwards = Multiply(new Quat(_camera.Rotation), -Vector3.Forward);
		motionDir = Multiply(m, forwards);
		sideDir = Multiply(new Quat(_camera.Rotation), -Vector3.Left);
		//sideDir = sideDir.Normalized();
		motionDir = upDir.Cross(sideDir);
	}

	private Vector3 _getExternalForces(float dt, List<CollisionInfo> contacts)
	{
		Vector3 force = GravityDir * Gravity;
		if (contacts.Count == 0)
		{
			_getMarbleAxis(out var sideDir, out var motionDir, out Vector3 _);
			force += (sideDir * InputMovement.x + motionDir * InputMovement.y) * AirAcceleration;
		}

		return force;
	}

	private void _velocityCancel(List<CollisionInfo> contacts, ref Vector3 velocity, ref Vector3 omega, bool surfaceSlide, bool noBounce)
	{
		bool flag1 = false;
		int iterations = 0;
		bool done = false;
		while (!done) {
			done = true;
			++iterations;
			foreach (var coll in contacts)
			{
				Vector3 relativeVelocity = velocity - coll.Velocity;
				float bounceSpeed = coll.Normal.Dot(relativeVelocity);
				if (!flag1 && bounceSpeed < 0.0 || bounceSpeed < -0.001f)
				{
					Vector3 invBounce = bounceSpeed * coll.Normal;
					// _reportBounce(contacts[index].point, contacts[index].normal, -num3);
					if (noBounce)
					{
						velocity -= invBounce;
					}
					else if (coll.Collider != null && false)
					{
						CollisionInfo contact = coll;
						//if (false && contact.Collider.GetComponent<Movement>() is Movement owner) {
						/*	float num5 = 1f;
							float num6 = 1f;
							float num7 = 0.5f;
							var vector3_4 =
							(
								(
									Vector3Helper.Dot(
										(
											(velocity * num5) -
											(owner.Velocity * num6)
										),
										contact.Normal
									) *
									contact.Normal
								) *
								(1f + num7)
							);

							Vector3.op_Multiply(
								Vector3.op_Multiply(
									Vector3Helper.Dot(
										Vector3.op_Subtraction(
											Vector3.op_Multiply(velocity, num5),
											Vector3.op_Multiply(owner.Velocity, num6)
										),
										contact.normal
									),
									contact.normal
								),
								1f + num7
							);
							velocity = velocity - (vector3_4 / num5);
							//velocity = Vector3.op_Subtraction(velocity, Vector3.op_Division(vector3_4, num5));
							var moveComponent = owner;
							moveComponent.Velocity = moveComponent.Velocity + (vector3_4 / num6);
							//moveComponent.Velocity = Vector3.op_Addition(moveComponent.Velocity, Vector3.op_Division(vector3_4, num6));
							contact.Velocity = owner.Velocity;
							//contacts[index] = contact; // ?*/
				//}
				//else
				//{
				float num5 = 0.5f;
							Vector3 vector34 = contact.Normal * (velocity.Dot(contact.Normal) * (1f + num5));
							velocity -= vector34;
							
						//}
					}
					else
					{
						if (coll.Velocity.Length() > 0.00001 && !surfaceSlide &&
							bounceSpeed > -MaxDotSlide * velocity.Length())
						{
							velocity -= invBounce;
							velocity = velocity.Normalized();
							velocity *= velocity.Length();
							surfaceSlide = true;
						}
						else if (bounceSpeed > -MinBounceVel)
						{
							velocity -= invBounce;
						}
						else
						{
							Vector3 velocityAdd = (float) -(1.0 + BounceRestitution * coll.Restitution) * invBounce;
							Vector3 velocityAtContact = relativeVelocity + omega.Cross(-coll.Normal * Radius);
							float num5 = -coll.Normal.Dot(relativeVelocity);
							Vector3 vector36 = velocityAtContact - coll.Normal * coll.Normal.Dot(relativeVelocity);
							float num6 = vector36.Length();
							if (Math.Abs(num6) > 0.001f)
							{
								float inertia = (float) (5.0 * (BounceKineticFriction * coll.Friction) * num5 /
													  (2.0 * Radius));
								if (inertia > num6 / Radius)
									inertia = num6 / Radius;
								Vector3 vector37 = vector36 / num6;
								Vector3 vAtC = -coll.Normal.Cross(-vector37);
								Vector3 vector38 = inertia * vAtC;
								omega += vector38;
								velocity -= -vector38.Cross(-coll.Normal * Radius);
							}

							velocity += velocityAdd;
						}
					}

					done = false;
				}
			}

			flag1 = true;
			if (iterations > 6 && noBounce)
				done = true;

			if (iterations > 1e7) {
				GD.Print("Collision lock");
				break;
			}
		}

		if (velocity.LengthSquared() >= 625.0)
			return;
		bool flag3 = false;
		Vector3 vector39 = new Vector3(0.0f, 0.0f, 0.0f);
		for (int index = 0; index < contacts.Count; ++index)
		{
			Vector3 vector32 = vector39 + contacts[index].Normal;
			if (vector32.LengthSquared() < 0.01)
				vector32 += contacts[index].Normal;
			vector39 = vector32;
			flag3 = true;
		}

		if (!flag3)
			return;
		vector39 = vector39.Normalized();
		float num8 = 0.0f;
		for (int index = 0; index < contacts.Count; ++index)
		{
			if (contacts[index].Penetration < Radius)
			{
				float num3 = 0.1f;
				float penetration = contacts[index].Penetration;
				float num4 = (velocity + num8 * vector39).Dot(contacts[index].Normal);
				if (num3 * num4 < penetration)
					num8 += (penetration - num4 * num3) / num3 / contacts[index].Normal.Dot(vector39);
			}
		}

		float num9 = Mathf.Clamp(num8, -25f, 25f);
		velocity += num9 * vector39;
	}


	private void _applyContactForces(
		float dt,
		List<CollisionInfo> contacts,
		bool isCentered,
		Vector3 aControl,
		Vector3 desiredOmega,
		ref Vector3 velocity,
		ref Vector3 omega,
		ref Vector3 linAccel,
		out Vector3 angAccel)
	{
		angAccel = Vector3.Zero;
		_slipAmount = 0.0f;
		Vector3 vector31 = GravityDir;
		int index1 = -1;
		float num1 = 0.0f;
		for (int index2 = 0; index2 < contacts.Count; ++index2)
		{
			// if (contacts[index2].collider == null)
			// {
			float num2 = -contacts[index2].Normal.Dot(linAccel);
			if (num2 > num1)
			{
				num1 = num2;
				index1 = index2;
			}

			// }
		}

		CollisionInfo collisionInfo = index1 != -1 ? contacts[index1] : new CollisionInfo();
		if (index1 != -1 && Jump)
		{
			Vector3 vector32 = velocity - collisionInfo.Velocity;
			float num2 = collisionInfo.Normal.Dot(vector32);
			if (num2 < 0.0)
				num2 = 0.0f;
			if (num2 < JumpImpulse)
			{
				velocity += collisionInfo.Normal * (JumpImpulse - num2);
				// MarbleControlComponent._soundBank.PlayCue(MarbleControlComponent._sounds[12]);
			}
		}

		for (int index2 = 0; index2 < contacts.Count; ++index2)
		{
			float num2 = -contacts[index2].Normal.Dot(linAccel);
			if (num2 > 0.0 && contacts[index2].Normal.Dot(velocity - contacts[index2].Velocity) <=
				0.00001)
			{
				linAccel += contacts[index2].Normal * num2;
			}
		}

		if (index1 != -1)
		{
			// (collisionInfo.velocity - (collisionInfo.normal * collisionInfo.normal.Dot(collisionInfo.velocity)));
			Vector3 vector32 = velocity + omega.Cross(-collisionInfo.Normal * Radius) - collisionInfo.Velocity;
			float num2 = vector32.Length();
			bool flag = false;
			Vector3 vector33 = new Vector3(0.0f, 0.0f, 0.0f);
			Vector3 vector34 = new Vector3(0.0f, 0.0f, 0.0f);
			if (num2 != 0.0)
			{
				flag = true;
				float num3 = KineticFriction * collisionInfo.Friction;
				float num4 = (float) (5.0 * num3 * num1 / (2.0 * Radius));
				float num5 = num1 * num3;
				float num6 = (num4 * Radius + num5) * dt;
				if (num6 > num2)
				{
					float num7 = num2 / num6;
					num4 *= num7;
					num5 *= num7;
					flag = false;
				}

				Vector3 vector35 = vector32 / num2;
				vector33 = num4 * -collisionInfo.Normal.Cross(-vector35);
				vector34 = -num5 * vector35;
				_slipAmount = num2 - num6;
			}

			if (!flag)
			{
				Vector3 vector35 = -vector31 * Radius;
				Vector3 vector36 = vector35.Cross(linAccel) / vector35.LengthSquared();
				if (isCentered)
				{
					Vector3 vector37 = omega + angAccel * dt;
					aControl = desiredOmega - vector37;
					float num3 = aControl.Length();
					if (num3 > BrakingAcceleration)
						aControl = aControl * BrakingAcceleration / num3;
				}

				Vector3 vector38 = -aControl.Cross(-collisionInfo.Normal * Radius);
				Vector3 vector39 = vector36.Cross(-collisionInfo.Normal * Radius) + vector38;
				float num4 = vector39.Length();
				float num5 = StaticFriction * collisionInfo.Friction;
				if (num4 > num5 * num1)
				{
					float num3 = KineticFriction * collisionInfo.Friction;
					vector38 *= num3 * num1 / num4;
				}

				linAccel += vector38;
				angAccel += vector36;
			}

			linAccel += vector34;
			angAccel += vector33;
		}

		angAccel += aControl;
	}

	// Rotates the point /point/ with /rotation/.
	public static Vector3 Multiply(Quat rotation, Vector3 point) {
		float x = rotation.x * 2F;
		float y = rotation.y * 2F;
		float z = rotation.z * 2F;
		float xx = rotation.x * x;
		float yy = rotation.y * y;
		float zz = rotation.z * z;
		float xy = rotation.x * y;
		float xz = rotation.x * z;
		float yz = rotation.y * z;
		float wx = rotation.w * x;
		float wy = rotation.w * y;
		float wz = rotation.w * z;

		Vector3 res;
		res.x = (1F - (yy + zz)) * point.x + (xy - wz) * point.y + (xz + wy) * point.z;
		res.y = (xy + wz) * point.x + (1F - (xx + zz)) * point.y + (yz - wx) * point.z;
		res.z = (xz - wy) * point.x + (yz + wx) * point.y + (1F - (xx + yy)) * point.z;
		return res;
	}

	static class CollisionHelpers
	{
		public static bool ClosestPtPointTriangle(
			Vector3 pt,
			float radius,
			Vector3 p0,
			Vector3 p1,
			Vector3 p2,
			Vector3 normal,
			out Vector3 closest)
		{
			closest = Vector3.Zero;
			float num1 = pt.Dot(normal);
			float num2 = p0.Dot(normal);
			if (Mathf.Abs(num1 - num2) > radius * 1.1)
				return false;
			closest = pt + (num2 - num1) * normal;
			if (PointInTriangle(closest, p0, p1, p2))
				return true;
			float num3 = 10f;
			if (IntersectSegmentCapsule(pt, pt, p0, p1, radius, out var tSeg, out var tCap) &&
				tSeg < num3)
			{
				closest = p0 + tCap * (p1 - p0);
				num3 = tSeg;
			}

			if (IntersectSegmentCapsule(pt, pt, p1, p2, radius, out tSeg, out tCap) &&
				tSeg < num3)
			{
				closest = p1 + tCap * (p2 - p1);
				num3 = tSeg;
			}

			if (IntersectSegmentCapsule(pt, pt, p2, p0, radius, out tSeg, out tCap) &&
				tSeg < num3)
			{
				closest = p2 + tCap * (p0 - p2);
				num3 = tSeg;
			}

			return num3 < 1.0;
		}

		public static bool PointInTriangle(Vector3 pnt, Vector3 a, Vector3 b, Vector3 c)
		{
			a -= pnt;
			b -= pnt;
			c -= pnt;
			Vector3 bc = b.Cross(c);
			Vector3 ca = c.Cross(a);
			if (bc.Dot(ca) < 0.0)
				return false;
			Vector3 ab = a.Cross(b);
			return bc.Dot(ab) >= 0.0;
		}

		public static bool IntersectSegmentCapsule(
			Vector3 segStart,
			Vector3 segEnd,
			Vector3 capStart,
			Vector3 capEnd,
			float radius,
			out float seg,
			out float cap)
		{
			return ClosestPtSegmentSegment(segStart, segEnd, capStart, capEnd, out seg, out cap,
				out Vector3 _, out Vector3 _) < radius * radius;
		}

		public static float ClosestPtSegmentSegment(
			Vector3 p1,
			Vector3 q1,
			Vector3 p2,
			Vector3 q2,
			out float s,
			out float T,
			out Vector3 c1,
			out Vector3 c2)
		{
			float num1 = 0.0001f;
			Vector3 vector31 = q1 - p1;
			Vector3 vector32 = q2 - p2;
			Vector3 vector33 = p1 - p2;
			float num2 = vector31.Dot(vector31);
			float num3 = vector32.Dot(vector32);
			float num4 = vector32.Dot(vector33);
			if (num2 <= num1 && num3 <= num1)
			{
				s = T = 0.0f;
				c1 = p1;
				c2 = p2;
				return (c1 - c2).Dot(c1 - c2);
			}

			if (num2 <= num1)
			{
				s = 0.0f;
				T = num4 / num3;
				T = Mathf.Clamp(T, 0.0f, 1f);
			}
			else
			{
				float num5 = vector31.Dot(vector33);
				if (num3 <= num1)
				{
					T = 0.0f;
					s = Mathf.Clamp(-num5 / num2, 0.0f, 1f);
				}
				else
				{
					float num6 = vector31.Dot(vector32);
					float num7 = (float) (num2 * num3 - num6 * num6);
					s = num7 == 0.0
						? 0.0f
						: Mathf.Clamp(
							(float) (num6 * num4 - num5 * num3) / num7, 0.0f, 1f);
					T = (num6 * s + num4) / num3;
					if (T < 0.0)
					{
						T = 0.0f;
						s = Mathf.Clamp(-num5 / num2, 0.0f, 1f);
					}
					else if (T > 1.0)
					{
						T = 1f;
						s = Mathf.Clamp((num6 - num5) / num2, 0.0f, 1f);
					}
				}
			}

			c1 = p1 + vector31 * s;
			c2 = p2 + vector32 * T;
			return (c1 - c2).Dot(c1 - c2);
		}
	}
}
