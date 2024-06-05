 using UnityEngine;
#if ENABLE_INPUT_SYSTEM 
using UnityEngine.InputSystem;
#endif

/* Note: animations are called via the controller for both the character and capsule using animator null checks
 */

namespace StarterAssets
{
    [RequireComponent(typeof(CharacterController))]
#if ENABLE_INPUT_SYSTEM 
    [RequireComponent(typeof(PlayerInput))]
#endif
    public class ThirdPersonController_Bounce : MonoBehaviour
    {
        [Header("Player")]
        [Tooltip("Move speed of the character in m/s")]
        public float MoveSpeed = 2.0f;

        [Tooltip("Sprint speed of the character in m/s")]
        public float SprintSpeed = 5.335f;

        [Tooltip("How fast the character turns to face movement direction")]
        public float RotationSmoothTime = 0.12f;

        [Tooltip("Acceleration and deceleration")]
        public float SpeedChangeRate = 10.0f;

        [Tooltip("Acceleration Tilt in Degrees")]
        public float tiltStrength = 50f;

        [Tooltip("Acceleration Tilt in Degrees")]
        public float TiltChangeRate = 1f;

        [Tooltip("Walk Wheel Radius")]
        public float WalkWheelRadius = 1f;

        [Tooltip("Sprint Wheel Radius")]
        public float SprintWheelRadius = 1f;

        public bool StrideWheelVisible = false;

        public bool trailVisible = false;

        public AudioClip LandingAudioClip;
        public AudioClip[] FootstepAudioClips;
        [Range(0, 1)] public float FootstepAudioVolume = 0.5f;

        [Space(10)]
        [Tooltip("The height the player can jump")]
        public float JumpHeight = 1.2f;

        [Tooltip("The character uses its own gravity value. The engine default is -9.81f")]
        public float Gravity = -15.0f;

        [Space(10)]
        [Tooltip("Time required to pass before being able to jump again. Set to 0f to instantly jump again")]
        public float JumpTimeout = 0.50f;

        [Tooltip("Time required to pass before entering the fall state. Useful for walking down stairs")]
        public float FallTimeout = 0.15f;

        [Header("Player Grounded")]
        [Tooltip("If the character is grounded or not. Not part of the CharacterController built in grounded check")]
        public bool Grounded = true;

        [Tooltip("Useful for rough ground")]
        public float GroundedOffset = -0.14f;

        [Tooltip("The radius of the grounded check. Should match the radius of the CharacterController")]
        public float GroundedRadius = 0.28f;

        [Tooltip("What layers the character uses as ground")]
        public LayerMask GroundLayers;

        [Header("Cinemachine")]
        [Tooltip("The follow target set in the Cinemachine Virtual Camera that the camera will follow")]
        public GameObject CinemachineCameraTarget;

        [Tooltip("How far in degrees can you move the camera up")]
        public float TopClamp = 70.0f;

        [Tooltip("How far in degrees can you move the camera down")]
        public float BottomClamp = -30.0f;

        [Tooltip("Additional degress to override the camera. Useful for fine tuning camera position when locked")]
        public float CameraAngleOverride = 0.0f;

        [Tooltip("For locking the camera position on all axis")]
        public bool LockCameraPosition = false;

        // cinemachine
        private float _cinemachineTargetYaw;
        private float _cinemachineTargetPitch;

        // player
        private Vector3 _previousHorizontalVelocity;
        private Vector3 _smoothedAcceleration = Vector3.zero;
        private float _rotation;
        private float _animationBlend;
        private float _rotationVelocity;
        private float _verticalVelocity;
        private float _terminalVelocity = -53.0f;
        private float _previousStrideWheelRotation = 0;
        private bool _bounce = false;
        private float _bounceOffset;
        private float _bounceSpeedMult;

        // timeout deltatime
        private float _jumpTimeoutDelta;
        private float _fallTimeoutDelta;

        // animation IDs
        private int _animIDSpeed;
        private int _animIDGrounded;
        private int _animIDJump;
        private int _animIDFreeFall;
        private int _animIDStride;

#if ENABLE_INPUT_SYSTEM 
        private PlayerInput _playerInput;
#endif
        private Animator _animator;
        private CharacterController _controller;
        private StarterAssetsInputs _input;
        private GameObject _mainCamera;
        [SerializeField] private GameObject _strideWheel;
        private Renderer _strideWheelRenderer;
        [SerializeField] private GameObject _trail;

        private const float _threshold = 0.01f;

        private bool _hasAnimator;

        private bool IsCurrentDeviceMouse
        {
            get
            {
#if ENABLE_INPUT_SYSTEM
                return _playerInput.currentControlScheme == "KeyboardMouse";
#else
                return false;
#endif
            }
        }


        private void Awake()
        {
            // get a reference to our main camera
            if (_mainCamera == null)
            {
                _mainCamera = GameObject.FindGameObjectWithTag("MainCamera");
            }
        }

        private void Start()
        {
            _cinemachineTargetYaw = CinemachineCameraTarget.transform.rotation.eulerAngles.y;

            _hasAnimator = TryGetComponent(out _animator);
            _controller = GetComponent<CharacterController>();
            _input = GetComponent<StarterAssetsInputs>();
#if ENABLE_INPUT_SYSTEM 
            _playerInput = GetComponent<PlayerInput>();
#else
            Debug.LogError( "Starter Assets package is missing dependencies. Please use Tools/Starter Assets/Reinstall Dependencies to fix it");
#endif

            AssignAnimationIDs();

            // reset our timeouts on start
            _jumpTimeoutDelta = JumpTimeout;
            _fallTimeoutDelta = FallTimeout;


            _strideWheel = Instantiate(_strideWheel, transform);
            _strideWheelRenderer = _strideWheel.GetComponent<Renderer>();

            _trail = Instantiate(_trail, transform);

        }

        private void Update()
        {
            _hasAnimator = TryGetComponent(out _animator);

            JumpAndGravity();
            GroundedCheck();
            Move();
        }

        private void LateUpdate()
        {
            CameraRotation();
        }

        private void AssignAnimationIDs()
        {
            _animIDSpeed = Animator.StringToHash("Speed");
            _animIDGrounded = Animator.StringToHash("Grounded");
            _animIDJump = Animator.StringToHash("Jump");
            _animIDFreeFall = Animator.StringToHash("FreeFall");
            _animIDStride = Animator.StringToHash("Stride");
        }

        private void GroundedCheck()
        {
            // set sphere position, with offset
            Vector3 spherePosition = transform.TransformPoint(_controller.center) + new Vector3(0, -_controller.center.y - GroundedOffset, 0);
            Grounded = Physics.CheckSphere(spherePosition, GroundedRadius, GroundLayers,
                QueryTriggerInteraction.Ignore);

            // update animator if using character
            if (_hasAnimator)
            {
                _animator.SetBool(_animIDGrounded, Grounded);
            }
        }

        private void CameraRotation()
        {
            // if there is an input and camera position is not fixed
            if (_input.look.sqrMagnitude >= _threshold && !LockCameraPosition)
            {
                //Don't multiply mouse input by Time.deltaTime;
                float deltaTimeMultiplier = IsCurrentDeviceMouse ? 1.0f : Time.deltaTime;

                _cinemachineTargetYaw += _input.look.x * deltaTimeMultiplier;
                _cinemachineTargetPitch += _input.look.y * deltaTimeMultiplier;
            }

            // clamp our rotations so our values are limited 360 degrees
            _cinemachineTargetYaw = ClampAngle(_cinemachineTargetYaw, float.MinValue, float.MaxValue);
            _cinemachineTargetPitch = ClampAngle(_cinemachineTargetPitch, BottomClamp, TopClamp);

            // Cinemachine will follow this target
            CinemachineCameraTarget.transform.rotation = Quaternion.Euler(_cinemachineTargetPitch + CameraAngleOverride,
                _cinemachineTargetYaw, 0.0f);
        }

        private void Move()
        {
            Vector3 currentHorizontalVelocity = new(_controller.velocity.x, 0, _controller.velocity.z);

            float currentStrideWheelRotation = GetStrideWheelRotation(_previousStrideWheelRotation, currentHorizontalVelocity.magnitude);

            if (_hasAnimator)
            {
                UpdateAnimator(currentHorizontalVelocity.magnitude, currentStrideWheelRotation);
            }

            UpdateBounce(_previousStrideWheelRotation, currentStrideWheelRotation, currentHorizontalVelocity.magnitude);

            ApplyYaw(currentHorizontalVelocity.normalized);

            CalculateAcceleration(_previousHorizontalVelocity, currentHorizontalVelocity);

            ApplyTilt(_smoothedAcceleration);

            float inputAngle = Mathf.Atan2(_input.move.x, _input.move.y) * Mathf.Rad2Deg + _mainCamera.transform.eulerAngles.y;

            Vector3 targetVelocity = GetTargetVelocity(inputAngle);

            Vector3 newHorizontalVelocity = GetNewVelocity(targetVelocity, currentHorizontalVelocity);
            MovePlayer(newHorizontalVelocity);

            _previousHorizontalVelocity = currentHorizontalVelocity;
            _previousStrideWheelRotation = currentStrideWheelRotation;

        }

        private float GetStrideWheelRotation(float previousStrideWheelRotation, float currentHorizontalSpeed)
        {
            _strideWheelRenderer.enabled = StrideWheelVisible;

            float lowerBound = 2;

            float t = (currentHorizontalSpeed - lowerBound) / ((SprintSpeed - lowerBound) * .6f);

            float radius = Mathf.Lerp(WalkWheelRadius, SprintWheelRadius, t);
            _strideWheel.transform.localScale = new Vector3(radius * 2, radius * 2, radius * 2);
            _strideWheel.transform.localPosition = new Vector3(0, radius, 0);

            float distanceMoved = currentHorizontalSpeed * Time.deltaTime;

            float circumference = radius * 2 * Mathf.PI;

            // Calculate how much the wheel should rotate based on distance moved
            float rotationAngle = distanceMoved / circumference * 360f;

            float currentStrideWheelRotation = previousStrideWheelRotation + rotationAngle;

            currentStrideWheelRotation %= 360f; // Ensure the angle stays within 0-360

            // Apply the rotation to the wheel
            _strideWheel.transform.localRotation = Quaternion.Euler(0f, 90, currentStrideWheelRotation);

            return currentStrideWheelRotation;
        }

        private void UpdateBounce(float previousStrideRotation, float currentStrideRotation, float currentHorizontalSpeed)
        {
            float speedOffset = 0.1f;
            if (previousStrideRotation % 180 < 45 && currentStrideRotation % 180 > 45)
            {
                _bounce = currentHorizontalSpeed > MoveSpeed + speedOffset && Grounded;
                _bounceSpeedMult = 1 - (currentHorizontalSpeed / SprintSpeed);
            }

            if (currentHorizontalSpeed < MoveSpeed + speedOffset)
            {
                _bounce = false;
                _bounceOffset -= 2f * Time.deltaTime;
                if (_bounceOffset < 0) _bounceOffset = 0;
            } else
            {
                _bounceOffset = _bounce ? Mathf.Abs(Mathf.Sin((currentStrideRotation + 135) * Mathf.Deg2Rad)) * (.5f * _bounceSpeedMult + 0.2f) : 0;
            }
        }

        private void ApplyYaw(Vector3 currentHorizontalDirection)
        {
            if (_input.move != Vector2.zero)
            {
                float targetRotation = Mathf.Atan2(currentHorizontalDirection.x, currentHorizontalDirection.z) * Mathf.Rad2Deg;
                _rotation = Mathf.SmoothDampAngle(transform.eulerAngles.y, targetRotation, ref _rotationVelocity, RotationSmoothTime);
            }

            transform.rotation = Quaternion.Euler(0, _rotation, 0);
        }

        private void CalculateAcceleration(Vector3 previousHorizontalVelocity, Vector3 currentHorizontalVelocity)
        {
            Vector3 acceleration = Vector3.zero;
            float speedDifference = Mathf.Abs(currentHorizontalVelocity.magnitude - previousHorizontalVelocity.magnitude);
            float directionDifference = (currentHorizontalVelocity.normalized - previousHorizontalVelocity.normalized).magnitude;

            if (speedDifference > 0.0001 || directionDifference > 0.001)
            {
                if (directionDifference > 0.001 && speedDifference <= 0.0001)
                {
                    // No significant difference in speed, just use direction
                    acceleration = (currentHorizontalVelocity - previousHorizontalVelocity.normalized * currentHorizontalVelocity.magnitude) / Time.deltaTime;
                }
                else if (directionDifference <= 0.001 && speedDifference > 0.0001)
                {
                    // No significant difference in direction, just use speed
                    acceleration = (currentHorizontalVelocity - currentHorizontalVelocity.normalized * previousHorizontalVelocity.magnitude) / Time.deltaTime;
                }
                else
                {
                    acceleration = (currentHorizontalVelocity - previousHorizontalVelocity) / Time.deltaTime;
                }
            }

            acceleration = acceleration.normalized * Mathf.Clamp(acceleration.magnitude, 0, SprintSpeed * SpeedChangeRate * 10);

            _smoothedAcceleration = Vector3.Lerp(_smoothedAcceleration, acceleration, Time.deltaTime * TiltChangeRate);
        }

        private void ApplyTilt(Vector3 smoothedAcceleration)
        {

            float tiltAngle = smoothedAcceleration.magnitude * tiltStrength;
            Vector3 tiltAxis = Vector3.Cross(Vector3.up, smoothedAcceleration.normalized);

            Vector3 localTiltAxis = transform.InverseTransformDirection(tiltAxis);
            Quaternion localTiltRotation = Quaternion.AngleAxis(tiltAngle, localTiltAxis);

            transform.rotation *= localTiltRotation;
        }

        private void UpdateAnimator(float speed, float stride)
        {
            _animationBlend = Mathf.Lerp(_animationBlend, speed, Time.deltaTime * SpeedChangeRate);
            if (_animationBlend < 0.01f) _animationBlend = 0f;

            _animator.SetFloat(_animIDSpeed, _animationBlend / SprintSpeed);

            _animator.SetFloat(_animIDStride, Mathf.Floor(stride / 90f) / 4);
        }

        private Vector3 GetTargetVelocity(float inputAngle)
        {
            if (_input.move == Vector2.zero)
                return Vector3.zero;

            float speed = _input.sprint ? SprintSpeed : MoveSpeed;
            Vector3 targetVelocity = speed * (Quaternion.Euler(0.0f, inputAngle, 0.0f) * Vector3.forward);

            if (_input.analogMovement)
                targetVelocity *= _input.move.magnitude;

            return targetVelocity;
        }


        private Vector3 GetNewVelocity(Vector3 targetVelocity, Vector3 previousHorizontalVelocity)
        {
            float speedOffset = 0.1f;
            if ((previousHorizontalVelocity - targetVelocity).magnitude > speedOffset)
            {
                return Vector3.Lerp(previousHorizontalVelocity, targetVelocity, Time.deltaTime * SpeedChangeRate);
            }
            else
            {
                return targetVelocity;
            }
        }

        private void MovePlayer(Vector3 newHorizontalVelocity)
        {
            _controller.Move(newHorizontalVelocity.normalized * (newHorizontalVelocity.magnitude * Time.deltaTime) + new Vector3(0.0f, _verticalVelocity, 0.0f) * Time.deltaTime);
        }

        private void OnAnimatorIK(int layerIndex)
        {
            Vector3 rootPosition = _animator.bodyPosition;
            rootPosition.y += _bounceOffset;
            _animator.bodyPosition = rootPosition;

            _trail.transform.position = rootPosition;
            _trail.SetActive(trailVisible);
        }

        private void JumpAndGravity()
        {
            if (Grounded)
            {
                // reset the fall timeout timer
                _fallTimeoutDelta = FallTimeout;

                // update animator if using character
                if (_hasAnimator)
                {
                    _animator.SetBool(_animIDJump, false);
                    _animator.SetBool(_animIDFreeFall, false);
                }

                // stop our velocity dropping infinitely when grounded
                if (_verticalVelocity < 0.0f)
                {
                    _verticalVelocity = -2f;
                }

                // Jump
                if (_input.jump && _jumpTimeoutDelta <= 0.0f)
                {
                    // the square root of H * -2 * G = how much velocity needed to reach desired height
                    _verticalVelocity = Mathf.Sqrt(JumpHeight * -2f * Gravity);

                    // update animator if using character
                    if (_hasAnimator)
                    {
                        _animator.SetBool(_animIDJump, true);
                    }
                }

                // jump timeout
                if (_jumpTimeoutDelta >= 0.0f)
                {
                    _jumpTimeoutDelta -= Time.deltaTime;
                }
            }
            else
            {
                // reset the jump timeout timer
                _jumpTimeoutDelta = JumpTimeout;

                // fall timeout
                if (_fallTimeoutDelta >= 0.0f)
                {
                    _fallTimeoutDelta -= Time.deltaTime;
                }
                else
                {
                    // update animator if using character
                    if (_hasAnimator)
                    {
                        _animator.SetBool(_animIDFreeFall, true);
                    }
                }

                // if we are not grounded, do not jump
                _input.jump = false;
            }

            // apply gravity over time if under terminal (multiply by delta time twice to linearly speed up over time)
            if (_verticalVelocity > _terminalVelocity)
            {
                _verticalVelocity += Gravity * Time.deltaTime;
            }
        }

        private static float ClampAngle(float lfAngle, float lfMin, float lfMax)
        {
            if (lfAngle < -360f) lfAngle += 360f;
            if (lfAngle > 360f) lfAngle -= 360f;
            return Mathf.Clamp(lfAngle, lfMin, lfMax);
        }

        private void OnDrawGizmosSelected()
        {
            if (!Application.isPlaying || !isActiveAndEnabled)
                return;
                
            Color transparentGreen = new Color(0.0f, 1.0f, 0.0f, 0.35f);
            Color transparentRed = new Color(1.0f, 0.0f, 0.0f, 0.35f);

            if (Grounded) Gizmos.color = transparentGreen;
            else Gizmos.color = transparentRed;

            // when selected, draw a gizmo in the position of, and matching radius of, the grounded collider
            Gizmos.DrawSphere(
                transform.TransformPoint(_controller.center) + new Vector3(0, -_controller.center.y - GroundedOffset, 0),
                GroundedRadius);

        }

        private void OnFootstep(AnimationEvent animationEvent)
        {
            if (animationEvent.animatorClipInfo.weight > 0.5f)
            {
                if (FootstepAudioClips.Length > 0)
                {
                    var index = Random.Range(0, FootstepAudioClips.Length);
                    AudioSource.PlayClipAtPoint(FootstepAudioClips[index], transform.TransformPoint(_controller.center), FootstepAudioVolume);
                }
            }
        }

        private void OnLand(AnimationEvent animationEvent)
        {
            if (animationEvent.animatorClipInfo.weight > 0.5f)
            {
                AudioSource.PlayClipAtPoint(LandingAudioClip, transform.TransformPoint(_controller.center), FootstepAudioVolume);
            }
        }

        public static float Remap(float value, float fromMin, float fromMax, float toMin, float toMax)
        {
            return Mathf.Lerp(toMin, toMax, Mathf.InverseLerp(fromMin, fromMax, value));
        }

    }
}