using Godot;
using System;
using System.Collections.Generic;
using System.Linq;

namespace MarblePhysicsGodot.Scripts
{
	class Gravity : Area {

		public enum GravityType {
			Planar,
			Spherical
		}

        
        public override void _Ready() {
            base._Ready();
            Connect("body_entered", this, nameof(OnCollisionEnter));
            Connect("body_exited", this, nameof(OnCollisionExit));
            //CallDeferred("addTestSphere");
        }


        private void addTestSphere() {
            testSphere = new Spatial();
            testSphere.Name = "TestSphere";
            var meshInstance = new MeshInstance {
                Mesh = new SphereMesh(),
                Scale = new Vector3(0.3f, 0.3f, 0.3f)
            };
            testSphere.AddChild(meshInstance);
            GetTree().Root.GetNode("Spatial").AddChild(testSphere);
        }

        [Export] public GravityType _GravityType = GravityType.Planar;

        private Spatial testSphere;
        private bool _enabled = false;
        private Movement _player;

        public override void _PhysicsProcess(float delta) {
            base._PhysicsProcess(delta);

            if (_enabled == false)
                return;

            _player.GravityDir = _player.Translation.DirectionTo(GetParent<Spatial>().Translation);
            //testSphere.Translation = _player.Translation + _player.GravityDir;
        }

        private void AddSphere(Vector3 position) {
			var root = GetTree().Root.GetNodeOrNull("Spatial/Collisions2");
			if (root != null) {
				var spatial = new Spatial {
					Translation = position
				};
				var meshInstance = new MeshInstance {
					Mesh = new SphereMesh(),
					Scale = new Vector3(0.2f, 0.2f, 0.2f)
				};
				spatial.AddChild(meshInstance);
				root.AddChild(spatial);
			}
		}

		private void OnCollisionEnter(KinematicBody body) {
            var player = body as Movement;
			if (player == null)
				return;

            if (_GravityType == GravityType.Planar) {
                player.GravityDir = Vector3.Up;
            }
            else {
                _player = player;
                _enabled = true;
            }
			//AddSphere(body.Translation);
		}


		private void OnCollisionExit(KinematicBody body) {
            var player = body as Movement;
			if (player == null)
				return;

            _player = null;
            _enabled = false;
            player.GravityDir = Vector3.Down;
			//AddSphere(body.Translation);
		}
	}
}
