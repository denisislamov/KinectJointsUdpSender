using System.Net;
using System.Net.Sockets;

namespace Microsoft.Samples.Kinect.BodyBasics
{
  using System;
  using System.Collections.Generic;
  using System.ComponentModel;
  using System.IO;
  using System.Windows;
  using System.Windows.Media;
  using Microsoft.Kinect;

  public partial class MainWindow : INotifyPropertyChanged
  {
    private const double HAND_SIZE             = 30;
    private const double JOINT_THICKNESS       =  3;
    private const double CLIP_BOUNDS_THICKNESS = 10;
    
    private const float INFERRED_Z_POSITION_CLAMP = 0.1f;
    
    private readonly Brush _handClosedBrush   = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));
    private readonly Brush _handOpenBrush     = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));
    private readonly Brush _handLassoBrush    = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));
    private readonly Brush _trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));
    private readonly Brush _inferredJointBrush = Brushes.Yellow;
    
    private readonly Pen _inferredBonePen = new Pen(Brushes.Gray, 1);

    private readonly DrawingGroup _drawingGroup;
    private readonly DrawingImage _imageSource;
    
    private KinectSensor              _kinectSensor;
    private readonly CoordinateMapper _coordinateMapper;
    private BodyFrameReader           _bodyFrameReader;
    
    private Body[] _bodies;
    private readonly List<Tuple<JointType, JointType>> _bones;
    
    private readonly int _displayWidth;
    private readonly int _displayHeight;
    private readonly List<Pen> _bodyColors;
    private string _statusText;

    private static IPAddress _remoteIpAddress = IPAddress.Parse("127.0.0.1");
    private static int _port = 5001;

    [Serializable]
    public class KinectJoint
    {
      public string JointType;
      public bool IsTracking;

      public float PositionX;
      public float PositionY;
      public float PositionZ;

      public float RotationX;
      public float RotationY;
      public float RotationZ;
      public float RotationW;

      public byte[] Serialize()
      {
        using (MemoryStream m = new MemoryStream())
        {
          using (BinaryWriter writer = new BinaryWriter(m))
          {
            writer.Write(JointType);
            writer.Write(IsTracking);

            writer.Write(PositionX);
            writer.Write(PositionY);
            writer.Write(PositionZ);

            writer.Write(RotationX);
            writer.Write(RotationY);
            writer.Write(RotationZ);
            writer.Write(RotationW);
          }
          return m.ToArray();
        }
      }
      public static KinectJoint Desserialize(byte[] data)
      {
        KinectJoint result = new KinectJoint();
        using (MemoryStream m = new MemoryStream(data))
        {
          using (BinaryReader reader = new BinaryReader(m))
          {
            result.JointType = reader.ReadString();
            result.IsTracking = reader.ReadBoolean();

            result.PositionX = reader.ReadSingle();
            result.PositionY = reader.ReadSingle();
            result.PositionZ = reader.ReadSingle();

            result.RotationX = reader.ReadSingle();
            result.RotationY = reader.ReadSingle();
            result.RotationZ = reader.ReadSingle();
            result.RotationW = reader.ReadSingle();
          }
        }
        return result;
      }
    }

    private readonly KinectJoint _kinectJointData = new KinectJoint();

    UdpClient _udpSender = new UdpClient();
    IPEndPoint _endPoint = new IPEndPoint(_remoteIpAddress, _port);
    
    public MainWindow()
    {
      _kinectSensor     = KinectSensor.GetDefault();
      _coordinateMapper = _kinectSensor.CoordinateMapper;
      
      FrameDescription frameDescription = _kinectSensor.DepthFrameSource.FrameDescription;

      _displayWidth  = frameDescription.Width;
      _displayHeight = frameDescription.Height;

      _bodyFrameReader = _kinectSensor.BodyFrameSource.OpenReader();
      _bones = new List<Tuple<JointType, JointType>>
      {
        new Tuple<JointType, JointType>(JointType.Head,          JointType.Neck),
        new Tuple<JointType, JointType>(JointType.Neck,          JointType.SpineShoulder),
        new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid),
        new Tuple<JointType, JointType>(JointType.SpineMid,      JointType.SpineBase),
        new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight),
        new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft),
        new Tuple<JointType, JointType>(JointType.SpineBase,     JointType.HipRight),
        new Tuple<JointType, JointType>(JointType.SpineBase,     JointType.HipLeft),
        new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight),
        new Tuple<JointType, JointType>(JointType.ElbowRight,    JointType.WristRight),
        new Tuple<JointType, JointType>(JointType.WristRight,    JointType.HandRight),
        new Tuple<JointType, JointType>(JointType.HandRight,     JointType.HandTipRight),
        new Tuple<JointType, JointType>(JointType.WristRight,    JointType.ThumbRight),
        new Tuple<JointType, JointType>(JointType.ShoulderLeft,  JointType.ElbowLeft),
        new Tuple<JointType, JointType>(JointType.ElbowLeft,     JointType.WristLeft),
        new Tuple<JointType, JointType>(JointType.WristLeft,     JointType.HandLeft),
        new Tuple<JointType, JointType>(JointType.HandLeft,      JointType.HandTipLeft),
        new Tuple<JointType, JointType>(JointType.WristLeft,     JointType.ThumbLeft),
        new Tuple<JointType, JointType>(JointType.HipRight,      JointType.KneeRight),
        new Tuple<JointType, JointType>(JointType.KneeRight,     JointType.AnkleRight),
        new Tuple<JointType, JointType>(JointType.AnkleRight,    JointType.FootRight),
        new Tuple<JointType, JointType>(JointType.HipLeft,       JointType.KneeLeft),
        new Tuple<JointType, JointType>(JointType.KneeLeft,      JointType.AnkleLeft),
        new Tuple<JointType, JointType>(JointType.AnkleLeft,     JointType.FootLeft)
      };

      _bodyColors = new List<Pen>
      {
        new Pen(Brushes.Red,    6),
        new Pen(Brushes.Orange, 6),
        new Pen(Brushes.Green,  6),
        new Pen(Brushes.Blue,   6),
        new Pen(Brushes.Indigo, 6),
        new Pen(Brushes.Violet, 6)
      };

      _kinectSensor.IsAvailableChanged += Sensor_IsAvailableChanged;
      _kinectSensor.Open();
      
      StatusText = _kinectSensor.IsAvailable ?
                   Properties.Resources.RunningStatusText :
                   Properties.Resources.NoSensorStatusText;

      _drawingGroup = new DrawingGroup();
      _imageSource  = new DrawingImage(_drawingGroup);

      DataContext = this;
      InitializeComponent();
    }

    public event PropertyChangedEventHandler PropertyChanged;
    public ImageSource ImageSource
    {
      get
      {
        return _imageSource;
      }
    }

    public string StatusText
    {
      get
      {
        return _statusText;
      }

      set
      {
        if (_statusText != value)
        {
          _statusText = value;

          if (PropertyChanged != null)
          {
            PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
          }
        }
      }
    }

    private void MainWindow_Loaded(object sender, RoutedEventArgs e)
    {
      if (_bodyFrameReader != null)
      {
        _bodyFrameReader.FrameArrived += Reader_FrameArrived;
      }
    }

    private void MainWindow_Closing(object sender, CancelEventArgs e)
    {
      if (_bodyFrameReader != null)
      {
        _bodyFrameReader.Dispose();
        _bodyFrameReader = null;
      }

      if (_kinectSensor != null)
      {
        _kinectSensor.Close();
        _kinectSensor = null;
      }
    }

    private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
    {
      bool dataReceived = false;

      using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
      {
        if (bodyFrame != null)
        {
          if (_bodies == null)
          {
            _bodies = new Body[bodyFrame.BodyCount];
          }

          bodyFrame.GetAndRefreshBodyData(_bodies);
          dataReceived = true;
        }
      }

      if (dataReceived)
      {
        using (DrawingContext dc = _drawingGroup.Open())
        {
          dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, _displayWidth, _displayHeight));

          int penIndex = 0;
          foreach (Body body in _bodies)
          {
            Pen drawPen = _bodyColors[penIndex++];

            if (body.IsTracked)
            {
              DrawClippedEdges(body, dc);

              var joints = body.Joints;
              var jointPoints = new Dictionary<JointType, Point>();

              foreach (JointType jointType in joints.Keys)
              {
                CameraSpacePoint position = joints[jointType].Position;

                _udpSender = new UdpClient();
                _endPoint  = new IPEndPoint(_remoteIpAddress, _port);
                
                if (_udpSender != null && _endPoint != null)
                {
                  try
                  {
                    _kinectJointData.JointType = jointType.ToString();
                    _kinectJointData.IsTracking = (joints[jointType].TrackingState == TrackingState.Tracked);

                    _kinectJointData.PositionX = joints[jointType].Position.X;
                    _kinectJointData.PositionY = joints[jointType].Position.Y;
                    _kinectJointData.PositionZ = joints[jointType].Position.Z;

                    _kinectJointData.RotationX = body.JointOrientations[jointType].Orientation.X;
                    _kinectJointData.RotationY = body.JointOrientations[jointType].Orientation.Y;
                    _kinectJointData.RotationZ = body.JointOrientations[jointType].Orientation.Z;
                    _kinectJointData.RotationW = body.JointOrientations[jointType].Orientation.W;

                    var bytes = _kinectJointData.Serialize();
                    _udpSender.Send(bytes, bytes.Length, _endPoint);
                  }
                  catch (Exception ex)
                  {
                    Console.WriteLine("Exception: " + ex.ToString() + "\n  " + ex.Message);
                  }
                  finally
                  {
                    _udpSender.Close();
                  }
                }

                if (position.Z < 0)
                {
                  position.Z = INFERRED_Z_POSITION_CLAMP;
                }

                DepthSpacePoint depthSpacePoint = _coordinateMapper.MapCameraPointToDepthSpace(position);
                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
              }

              DrawBody(joints, jointPoints, dc, drawPen);
              DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
              DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
            }
          }

          _drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, _displayWidth, _displayHeight));
        }
      }
    }

    private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints,
      DrawingContext drawingContext, Pen drawingPen)
    {
      foreach (var bone in _bones)
      {
        DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
      }

      foreach (JointType jointType in joints.Keys)
      {
        Brush drawBrush = null;

        TrackingState trackingState = joints[jointType].TrackingState;

        switch (trackingState)
        {
          case TrackingState.Tracked:
            drawBrush = _trackedJointBrush;
            break;
          case TrackingState.Inferred:
            drawBrush = _inferredJointBrush;
            break;
          case TrackingState.NotTracked:
            break;
        }

        if (drawBrush != null)
        {
          drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JOINT_THICKNESS, JOINT_THICKNESS);
        }
      }
    }

    private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
    {
      Joint joint0 = joints[jointType0];
      Joint joint1 = joints[jointType1];

      if (joint0.TrackingState == TrackingState.NotTracked || joint1.TrackingState == TrackingState.NotTracked)
      {
        return;
      }

      Pen drawPen = _inferredBonePen;
      if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
      {
        drawPen = drawingPen;
      }

      drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
    }

    private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
    {
      switch (handState)
      {
        case HandState.Closed:
          drawingContext.DrawEllipse(_handClosedBrush, null, handPosition, HAND_SIZE, HAND_SIZE);
          break;
        case HandState.Open:
          drawingContext.DrawEllipse(_handOpenBrush, null, handPosition, HAND_SIZE, HAND_SIZE);
          break;
        case HandState.Lasso:
          drawingContext.DrawEllipse(_handLassoBrush, null, handPosition, HAND_SIZE, HAND_SIZE);
          break;
        case HandState.Unknown:
          break;
        case HandState.NotTracked:
          break;
      }
    }

    private void DrawClippedEdges(Body body, DrawingContext drawingContext)
    {
      FrameEdges clippedEdges = body.ClippedEdges;

      if (clippedEdges.HasFlag(FrameEdges.Bottom))
      {
        drawingContext.DrawRectangle(Brushes.Red, null, new Rect(0, _displayHeight - CLIP_BOUNDS_THICKNESS, _displayWidth, CLIP_BOUNDS_THICKNESS));
      }

      if (clippedEdges.HasFlag(FrameEdges.Top))
      {
        drawingContext.DrawRectangle(Brushes.Red, null, new Rect(0, 0, _displayWidth, CLIP_BOUNDS_THICKNESS));
      }

      if (clippedEdges.HasFlag(FrameEdges.Left))
      {
        drawingContext.DrawRectangle(Brushes.Red, null, new Rect(0, 0, CLIP_BOUNDS_THICKNESS, _displayHeight));
      }

      if (clippedEdges.HasFlag(FrameEdges.Right))
      {
        drawingContext.DrawRectangle(Brushes.Red, null, new Rect(_displayWidth - CLIP_BOUNDS_THICKNESS, 0, CLIP_BOUNDS_THICKNESS, _displayHeight));
      }
    }

    private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
    {
      StatusText = _kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText : Properties.Resources.SensorNotAvailableStatusText;
    }

    private void Button_Click(object sender, RoutedEventArgs e)
    {
      _udpSender.Close();

      _remoteIpAddress = IPAddress.Parse(TextBox_IP.Text);
      _port = int.Parse(TextBox_Port.Text);

      _udpSender = new UdpClient();
      _endPoint  = new IPEndPoint(_remoteIpAddress, _port);
    }
  }
}
