using System;
using OpenTK.Mathematics;
using BulletSharp;

// Создаем псевдонимы, чтобы не путать типы OpenTK и BulletSharp
using BVector3 = BulletSharp.Math.Vector3;
using BMatrix = BulletSharp.Math.Matrix;
using BQuaternion = BulletSharp.Math.Quaternion;

namespace Tiny3DEngine
{
    public class CubePhysics : IDisposable
    {
        // Внутренние поля используем типы OpenTK для удобства движка
        private Vector3 _position = new(0, 3, 0);
        private Quaternion _rotation = Quaternion.Identity;
        private Vector3 _linearVelocity = Vector3.Zero;
        private Vector3 _angularVelocity = Vector3.Zero;

        private DiscreteDynamicsWorld? _dynamicsWorld;
        private RigidBody? _cubeRigidBody;
        private RigidBody? _groundRigidBody;
        private CollisionShape? _cubeCollisionShape;
        private CollisionShape? _groundCollisionShape;
        private DefaultCollisionConfiguration? _collisionConfig;
        private CollisionDispatcher? _dispatcher;
        private DbvtBroadphase? _broadphase;
        private SequentialImpulseConstraintSolver? _solver;

        private readonly float cubeHalfSize = 0.5f;
        public bool IsGrounded { get; private set; }
        private bool _disposed = false;

        public Vector3 Position { get => _position; set => _position = value; }
        public Quaternion Rotation { get => _rotation; set => _rotation = value; }
        public Vector3 LinearVelocity { get => _linearVelocity; set => _linearVelocity = value; }
        public Vector3 AngularVelocity { get => _angularVelocity; set => _angularVelocity = value; }

        public void Start()
        {
            InitializeBulletPhysics();
        }

        private void InitializeBulletPhysics()
        {
            _collisionConfig = new DefaultCollisionConfiguration();
            _dispatcher = new CollisionDispatcher(_collisionConfig);
            _broadphase = new DbvtBroadphase();
            _solver = new SequentialImpulseConstraintSolver();

            _dynamicsWorld = new DiscreteDynamicsWorld(_dispatcher, _broadphase, _solver, _collisionConfig)
            {
                // Используем BVector3 (тип из BulletSharp)
                Gravity = new BVector3(0, -9.81f, 0)
            };

            _cubeCollisionShape = new BoxShape(cubeHalfSize, cubeHalfSize, cubeHalfSize);

            // Создаем матрицу трансформации (BulletSharp.Math)
            var startTransform = BMatrix.Translation(0, 3, 0);

            var cubeMotionState = new DefaultMotionState(startTransform);
            var cubeInertia = _cubeCollisionShape.CalculateLocalInertia(1.0f);
            var cubeConstructionInfo = new RigidBodyConstructionInfo(1.0f, cubeMotionState, _cubeCollisionShape, cubeInertia);

            _cubeRigidBody = new RigidBody(cubeConstructionInfo)
            {
                Restitution = 0.6f,
                Friction = 0.5f,
                RollingFriction = 0.1f
            };

            _dynamicsWorld.AddRigidBody(_cubeRigidBody);
            CreateGround();
        }

        private void CreateGround()
        {
            _groundCollisionShape = new StaticPlaneShape(new BVector3(0, 1, 0), -2.0f);
            var groundMotionState = new DefaultMotionState();
            var groundConstructionInfo = new RigidBodyConstructionInfo(0, groundMotionState, _groundCollisionShape);
            _groundRigidBody = new RigidBody(groundConstructionInfo)
            {
                Restitution = 0.4f,
                Friction = 0.6f
            };
            _dynamicsWorld.AddRigidBody(_groundRigidBody);
        }

        public void Update(float deltaTime)
        {
            _dynamicsWorld?.StepSimulation(deltaTime);

            if (_cubeRigidBody != null && _cubeRigidBody.MotionState != null)
            {
                _cubeRigidBody.MotionState.GetWorldTransform(out BMatrix transform);

                // Конвертация BulletSharp Matrix -> OpenTK Vector3 & Quaternion
                _position = new Vector3(transform.Origin.X, transform.Origin.Y, transform.Origin.Z);
                _rotation = MatrixToQuaternion(transform);

                var linVel = _cubeRigidBody.LinearVelocity;
                var angVel = _cubeRigidBody.AngularVelocity;

                _linearVelocity = new Vector3(linVel.X, linVel.Y, linVel.Z);
                _angularVelocity = new Vector3(angVel.X, angVel.Y, angVel.Z);
            }

            CheckGrounded();
        }

        // Метод конвертации Матрицы BulletSharp в Кватернион OpenTK
        private static Quaternion MatrixToQuaternion(BMatrix matrix)
        {
            // Получаем масштаб (Scale), чтобы нормализовать матрицу вращения
            Vector3 scale = new Vector3(
                new Vector3(matrix.M11, matrix.M12, matrix.M13).Length,
                new Vector3(matrix.M21, matrix.M22, matrix.M23).Length,
                new Vector3(matrix.M31, matrix.M32, matrix.M33).Length
            );

            // Убираем масштаб из матрицы для чистого вращения
            float m11 = matrix.M11 / scale.X;
            float m22 = matrix.M22 / scale.Y;
            float m33 = matrix.M33 / scale.Z;

            float trace = m11 + m22 + m33;
            Quaternion q = new Quaternion();

            if (trace > 0)
            {
                float s = 0.5f / MathF.Sqrt(trace + 1.0f);
                q.W = 0.25f / s;
                q.X = (matrix.M32 / scale.Z - matrix.M23 / scale.Y) * s;
                q.Y = (matrix.M13 / scale.X - matrix.M31 / scale.Z) * s;
                q.Z = (matrix.M21 / scale.Y - matrix.M12 / scale.X) * s;
            }
            else
            {
                if (m11 > m22 && m11 > m33)
                {
                    float s = 2.0f * MathF.Sqrt(1.0f + m11 - m22 - m33);
                    q.W = (matrix.M32 / scale.Z - matrix.M23 / scale.Y) / s;
                    q.X = 0.25f * s;
                    q.Y = (matrix.M12 / scale.X + matrix.M21 / scale.Y) / s;
                    q.Z = (matrix.M13 / scale.X + matrix.M31 / scale.Z) / s;
                }
                else if (m22 > m33)
                {
                    float s = 2.0f * MathF.Sqrt(1.0f + m22 - m11 - m33);
                    q.W = (matrix.M13 / scale.X - matrix.M31 / scale.Z) / s;
                    q.X = (matrix.M12 / scale.X + matrix.M21 / scale.Y) / s;
                    q.Y = 0.25f * s;
                    q.Z = (matrix.M23 / scale.Y + matrix.M32 / scale.Z) / s;
                }
                else
                {
                    float s = 2.0f * MathF.Sqrt(1.0f + m33 - m11 - m22);
                    q.W = (matrix.M21 / scale.Y - matrix.M12 / scale.X) / s;
                    q.X = (matrix.M13 / scale.X + matrix.M31 / scale.Z) / s;
                    q.Y = (matrix.M23 / scale.Y + matrix.M32 / scale.Z) / s;
                    q.Z = 0.25f * s;
                }
            }
            return q;
        }

        private void CheckGrounded()
        {
            IsGrounded = false;
            if (_position.Y <= -1.9f && Math.Abs(_linearVelocity.Y) < 0.5f)
            {
                IsGrounded = true;
            }
        }

        public void AddImpulse(Vector3 impulse)
        {
            var bulletImpulse = new BVector3(impulse.X, impulse.Y, impulse.Z);
            _cubeRigidBody?.ApplyCentralImpulse(bulletImpulse);
            _cubeRigidBody?.Activate();
        }

        public void AddAngularImpulse(Vector3 impulse)
        {
            var bulletImpulse = new BVector3(impulse.X, impulse.Y, impulse.Z);
            _cubeRigidBody?.ApplyTorqueImpulse(bulletImpulse);
            _cubeRigidBody?.Activate();
        }

        public void AddRandomImpulse()
        {
            Random rand = new();
            Vector3 linearImpulse = new(
                (float)(rand.NextDouble() - 0.5) * 3f,
                5f + (float)rand.NextDouble() * 3f,
                (float)(rand.NextDouble() - 0.5) * 3f
            );
            Vector3 angularImpulse = new(
                (float)(rand.NextDouble() - 0.5) * 1f,
                (float)(rand.NextDouble() - 0.5) * 1f,
                (float)(rand.NextDouble() - 0.5) * 1f
            );
            AddImpulse(linearImpulse);
            AddAngularImpulse(angularImpulse);
        }

        public void SetPosition(Vector3 position)
        {
            if (_cubeRigidBody == null) return;

            var transform = _cubeRigidBody.WorldTransform;
            transform.Origin = new BVector3(position.X, position.Y, position.Z);
            _cubeRigidBody.WorldTransform = transform;
            _cubeRigidBody.LinearVelocity = BVector3.Zero;
            _cubeRigidBody.AngularVelocity = BVector3.Zero;
            _cubeRigidBody.ClearForces();
        }

        public void ResetPhysics()
        {
            SetPosition(new Vector3(0, 3, 0));
        }

        public void WakeUp()
        {
            _cubeRigidBody?.Activate();
        }

        public Matrix4 GetModelMatrix()
        {
            if (_cubeRigidBody == null) return Matrix4.Identity;

            _cubeRigidBody.MotionState.GetWorldTransform(out BMatrix transform);

            // Ручная конвертация матрицы 4x4 из Bullet в OpenTK
            return new Matrix4(
                transform.M11, transform.M12, transform.M13, transform.M14,
                transform.M21, transform.M22, transform.M23, transform.M24,
                transform.M31, transform.M32, transform.M33, transform.M34,
                transform.M41, transform.M42, transform.M43, transform.M44
            );
        }

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool disposing)
        {
            if (!_disposed)
            {
                if (disposing)
                {
                    if (_dynamicsWorld != null)
                    {
                        _dynamicsWorld.RemoveRigidBody(_cubeRigidBody);
                        _dynamicsWorld.RemoveRigidBody(_groundRigidBody);
                        _dynamicsWorld.Dispose();
                    }
                    _cubeRigidBody?.Dispose();
                    _groundRigidBody?.Dispose();
                    _cubeCollisionShape?.Dispose();
                    _groundCollisionShape?.Dispose();
                    _dispatcher?.Dispose();
                    _broadphase?.Dispose();
                    _solver?.Dispose();
                    _collisionConfig?.Dispose();
                }
                _disposed = true;
            }
        }

        ~CubePhysics()
        {
            Dispose(false);
        }
    }
}