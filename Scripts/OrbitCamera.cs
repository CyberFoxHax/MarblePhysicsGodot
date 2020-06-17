using Godot;

public class OrbitCamera : Camera {

	[Export] public float distance = 3;
	public float yaw;
	public float pitch;
	[Export] public float cameraSpeed = 120;
	[Export] public bool invertX;
	[Export] public bool invertY;

	[Export]
	private NodePath Target;
	private Spatial _target;

	public override void _Ready() {
		_target = GetNode<Spatial>(Target);
	}

	/*public RangeAttribute pitchRange;
	public RangeAttribute yawRange;*/

	private Vector2 _mouse;
	private bool _hasInput = false;

	public override void _Input(InputEvent evt) {
		if (evt is InputEventMouseMotion mouseMotion) {
			_mouse = mouseMotion.Relative / 100f;
			_mouse.x = -_mouse.x;
			_hasInput = true;
		}
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

    public override void _Process(float delta) {
        base._Process(delta);
        if(_target is Movement movement)
            Transform = Transform.LookingAt(movement.Translation, -movement.GravityDir);
    }

    public override void _PhysicsProcess(float delta) {
		if (_target == null)
			return;
		
		if (Input.GetMouseButtonMask() > 0) {
			Input.SetMouseMode(Input.MouseMode.Captured);
		}
		if (_hasInput == false)
			_mouse = Vector2.Zero;

		if (Input.GetMouseMode() == Input.MouseMode.Captured && _hasInput) {
			_hasInput = false;
			//Get updates from the input
			var joystick = new Vector2(
				Input.GetJoyAxis(0, 0),
				Input.GetJoyAxis(0, 1)
			);

			var input = joystick;
			if (input.LengthSquared() > 0 == false) {
				input = _mouse;
			}
			
			input *= delta;

			yaw += cameraSpeed * input.x * (invertX ? -1 : 1);
			pitch += cameraSpeed * input.y * (invertY ? 1 : -1);
		}

		float actualDistance = distance;// + (target.GetComponent<Marble>().radius * 2.0f);

		//Easy lock to the object
		Vector3 position = _target.Translation;

		//Rotate by pitch and yaw (and not roll, oh god my stomach)
		var rotation = new Quat(Vector3.Up, yaw);
		rotation *= new Quat(Vector3.Right, pitch);

		//Offset for orbiting
		position += Multiply(rotation, new Vector3(0.0f, 0.0f, actualDistance));

		var euler = rotation.GetEuler();

		//Lame way of updating the transform
		Rotation = new Vector3(euler.x, euler.y, 0);
		Translation = position;
	}
}
