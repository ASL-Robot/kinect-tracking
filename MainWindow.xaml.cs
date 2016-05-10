//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.BodyBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Forms;
    using System.Windows.Media;
    using System.Windows.Media.Media3D;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using Microsoft.Kinect.Face;
    using System.Windows.Shapes;
    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;
        string fileName = "sample";
        DepthSpacePoint[] facePoint;
        /// <summary>
        /// HighDefinitionFaceFrameSource to get a reader and a builder from.
        /// Also to set the currently tracked user id to get High Definition Face Frames of
        /// </summary>
        private HighDefinitionFaceFrameSource highDefinitionFaceFrameSource = null;

        /// <summary>
        /// HighDefinitionFaceFrameReader to read HighDefinitionFaceFrame to get FaceAlignment
        /// </summary>
        private HighDefinitionFaceFrameReader highDefinitionFaceFrameReader = null;

        /// <summary>
        /// FaceAlignment is the result of tracking a face, it has face animations location and orientation
        /// </summary>
        private FaceAlignment currentFaceAlignment = null;

        /// <summary>
        /// FaceModel is a result of capturing a face
        /// </summary>
        private FaceModel currentFaceModel = null;

        /// <summary>
        /// FaceModelBuilder is used to produce a FaceModel
        /// </summary>
        private FaceModelBuilder faceModelBuilder = null;

        /// <summary>
        /// The currently tracked body
        /// </summary>
        private ulong currentTrackingId = 0;

        /// <summary>
        /// The currently tracked body
        /// </summary>
        private Body currentTrackedBody = null;
        private bool onCapture = false;
        Timer timer = new Timer();
        Stopwatch stopWatch = new Stopwatch();

        private ulong CurrentTrackingId
        {
            get
            {
                return this.currentTrackingId;
            }

            set
            {
                this.currentTrackingId = value;
            }
        }
        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {

            File.WriteAllText(AppDomain.CurrentDomain.BaseDirectory + @"\signData\" + fileName + "_body.txt", "");
            File.WriteAllText(AppDomain.CurrentDomain.BaseDirectory + @"\signData\" + fileName + "_face.txt", "");
            File.WriteAllText(AppDomain.CurrentDomain.BaseDirectory + @"\signData\" + fileName + "_timer.txt", "");
            // one sensor is currently supported
            Console.Write(AppDomain.CurrentDomain.BaseDirectory);
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();
            addFile.IsEnabled = false;
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
                this.highDefinitionFaceFrameSource = new HighDefinitionFaceFrameSource(this.kinectSensor);
                this.highDefinitionFaceFrameSource.TrackingIdLost += this.HdFaceSource_TrackingIdLost;

                this.highDefinitionFaceFrameReader = this.highDefinitionFaceFrameSource.OpenReader();
                this.highDefinitionFaceFrameReader.FrameArrived += this.HdFaceReader_FrameArrived;
                this.currentFaceModel = new FaceModel();
                this.currentFaceAlignment = new FaceAlignment();
                var vertices = this.currentFaceModel.CalculateVerticesForAlignment(this.currentFaceAlignment);

            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
            if (this.currentFaceModel != null)
            {
                this.currentFaceModel.Dispose();
                this.currentFaceModel = null;
            }
        }

        public void onButtonClick(object sender, RoutedEventArgs e)
        {
            onCapture = !onCapture;
            if (onCapture == true)
            {
                File.WriteAllText(AppDomain.CurrentDomain.BaseDirectory + @"\signData\" + fileName + "_body.txt", "");
                File.WriteAllText(AppDomain.CurrentDomain.BaseDirectory + @"\signData\" + fileName + "_face.txt", "");
                File.WriteAllText(AppDomain.CurrentDomain.BaseDirectory + @"\signData\" + fileName + "_timer.txt", "");
                stopWatch.Reset();
                capture.Content = "Stop Capture";
                stopWatch.Start();
                timer.Interval = 1;
                timer.Tick += new EventHandler(OnTimedEvent);
                timer.Enabled = true;

            }
            else {
                stopWatch.Stop();
                timer.Enabled = false;
                capture.Content = "Start Capture";
            }
        }
        public void onAddingFile(object sender, RoutedEventArgs e)
        {
            if (InputTextBox.Text != null && InputTextBox.Text != String.Empty)
            {
                fileName = InputTextBox.Text;
                File.WriteAllText(AppDomain.CurrentDomain.BaseDirectory + @"\signData\" + fileName + "_body.txt", "");
                File.WriteAllText(AppDomain.CurrentDomain.BaseDirectory + @"\signData\" + fileName + "_face.txt", "");
                File.WriteAllText(AppDomain.CurrentDomain.BaseDirectory + @"\signData\" + fileName + "_timer.txt", "");
                addFile.IsEnabled = false;
            }
        }

        public void TextBox_MouseDown(object sender, RoutedEventArgs e)
        {
            addFile.IsEnabled = true;
        }
        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            //   bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    using (DrawingContext dc = this.drawingGroup.Open())
                    {
                        // Draw a transparent background to set the render size
                        dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                        int penIndex = 0;
                        foreach (Body body in this.bodies)
                        {
                            Pen drawPen = this.bodyColors[penIndex++];
                            if(body!=null){ 
                            if (body.IsTracked)
                            {
                                this.DrawClippedEdges(body, dc);

                                IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                                // convert the joint points to depth (display) space
                                Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();
                                int i = 0;
                                int j = 0;
                                string[] lines = new string[15];
                                foreach (JointType jointType in joints.Keys)
                                {

                                    // sometimes the depth(Z) of an inferred joint may show as negative
                                    // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                    CameraSpacePoint position = joints[jointType].Position;
                                    if (position.Z < 0)
                                    {
                                        position.Z = InferredZPositionClamp;
                                    }
                                    DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                    jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                                    if ((i >= 2 && i <= 11) || (i >= 20 && i <= 24))
                                    {
                                        lines[j] = joints[jointType].Position.X + "\t" + joints[jointType].Position.Y + "\t" + joints[jointType].Position.Z;
                                        j++;
                                    }
                                    i++;
                                }
                                if (onCapture == true)
                                {
                                    string[] timeLines = { stopWatch.ElapsedMilliseconds.ToString() };
                                    File.AppendAllLines(AppDomain.CurrentDomain.BaseDirectory + @"\signData\" + fileName + "_body.txt", lines);
                                    File.AppendAllLines(AppDomain.CurrentDomain.BaseDirectory + @"\signData\" + fileName + "_timer.txt", timeLines);
                                }
                                this.DrawBody(joints, jointPoints, dc, drawPen);
                                this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                                this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                            }
                          }
                        }

                        // prevent drawing outside of our render area
                        this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                        if (facePoint != null && facePoint.Length > 0)
                        {
                            dc.DrawEllipse(null, new Pen(Brushes.Blue, 4), new Point((facePoint[(int)HighDetailFacePoints.LefteyeInnercorner].X + facePoint[(int)HighDetailFacePoints.LefteyeOutercorner].X) / 2, (facePoint[(int)HighDetailFacePoints.LefteyeInnercorner].Y + facePoint[(int)HighDetailFacePoints.LefteyeOutercorner].Y) / 2), 1.0, 1.0);
                            dc.DrawEllipse(null, new Pen(Brushes.Blue, 4), new Point((facePoint[(int)HighDetailFacePoints.RighteyeInnercorner].X + facePoint[(int)HighDetailFacePoints.RighteyeOutercorner].X) / 2, (facePoint[(int)HighDetailFacePoints.RighteyeInnercorner].Y + facePoint[(int)HighDetailFacePoints.RighteyeOutercorner].Y) / 2), 1.0, 1.0);
                            dc.DrawEllipse(null, new Pen(Brushes.Blue, 4), new Point(facePoint[(int)HighDetailFacePoints.NoseTip].X, facePoint[(int)HighDetailFacePoints.NoseTip].Y), 1.0, 1.0);
                            dc.DrawEllipse(null, new Pen(Brushes.Blue, 3), new Point(facePoint[(int)HighDetailFacePoints.MouthLeftcorner].X, facePoint[(int)HighDetailFacePoints.MouthLeftcorner].Y), 1.0, 1.0);
                            dc.DrawEllipse(null, new Pen(Brushes.Blue, 3), new Point(facePoint[(int)HighDetailFacePoints.MouthRightcorner].X, facePoint[(int)HighDetailFacePoints.MouthRightcorner].Y), 1.0, 1.0);
                            dc.DrawEllipse(null, new Pen(Brushes.Blue, 3), new Point(facePoint[(int)HighDetailFacePoints.MouthUpperlipMidtop].X, facePoint[(int)HighDetailFacePoints.MouthUpperlipMidtop].Y), 1.0, 1.0);
                            dc.DrawEllipse(null, new Pen(Brushes.Blue, 3), new Point(facePoint[(int)HighDetailFacePoints.MouthUpperlipMidbottom].X, facePoint[(int)HighDetailFacePoints.MouthUpperlipMidbottom].Y), 1.0, 1.0);
                            dc.DrawEllipse(null, new Pen(Brushes.Blue, 3), new Point(facePoint[(int)HighDetailFacePoints.MouthLowerlipMidtop].X, facePoint[(int)HighDetailFacePoints.MouthLowerlipMidtop].Y), 1.0, 1.0);
                            dc.DrawEllipse(null, new Pen(Brushes.Blue, 3), new Point(facePoint[(int)HighDetailFacePoints.MouthLowerlipMidbottom].X, facePoint[(int)HighDetailFacePoints.MouthLowerlipMidbottom].Y), 1.0, 1.0);

                        }

                    }
                    if (this.currentTrackedBody != null)
                    {
                        this.currentTrackedBody = FindBodyWithTrackingId(bodyFrame, this.CurrentTrackingId);

                        if (this.currentTrackedBody != null)
                        {
                            return;
                        }
                    }

                    Body selectedBody = FindClosestBody(bodyFrame);

                    if (selectedBody == null)
                    {
                        return;
                    }

                    this.currentTrackedBody = selectedBody;
                    this.CurrentTrackingId = selectedBody.TrackingId;

                    this.highDefinitionFaceFrameSource.TrackingId = this.CurrentTrackingId;

                }
            }
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }


        private void HdFaceSource_TrackingIdLost(object sender, TrackingIdLostEventArgs e)
        {
            var lostTrackingID = e.TrackingId;

            if (this.CurrentTrackingId == lostTrackingID)
            {
                this.CurrentTrackingId = 0;
                if (this.faceModelBuilder != null)
                {
                    this.faceModelBuilder.Dispose();
                    this.faceModelBuilder = null;
                }

                this.highDefinitionFaceFrameSource.TrackingId = 0;
            }
        }


        private void HdFaceReader_FrameArrived(object sender, HighDefinitionFaceFrameArrivedEventArgs e)
        {
            using (var frame = e.FrameReference.AcquireFrame())
            {
                // We might miss the chance to acquire the frame; it will be null if it's missed.
                // Also ignore this frame if face tracking failed.
                if (frame == null || !frame.IsFaceTracked)
                {
                    return;
                }

                frame.GetAndRefreshFaceAlignmentResult(this.currentFaceAlignment);
                var vertices = this.currentFaceModel.CalculateVerticesForAlignment(this.currentFaceAlignment);
                string[] lines ={
                    vertices[(int)HighDetailFacePoints.ForeheadCenter].X + "\t" + vertices[(int)HighDetailFacePoints.ForeheadCenter].Y + "\t" + vertices[(int)HighDetailFacePoints.ForeheadCenter].Z,
                    vertices[(int)HighDetailFacePoints.LefteyeInnercorner].X + "\t" + vertices[(int)HighDetailFacePoints.LefteyeInnercorner].Y + "\t" + vertices[(int)HighDetailFacePoints.LefteyeInnercorner].Z,
                    vertices[(int)HighDetailFacePoints.LefteyeOutercorner].X + "\t" + vertices[(int)HighDetailFacePoints.LefteyeOutercorner].Y + "\t" + vertices[(int)HighDetailFacePoints.LefteyeOutercorner].Z,
                    vertices[(int)HighDetailFacePoints.RighteyeInnercorner].X + "\t" + vertices[(int)HighDetailFacePoints.RighteyeInnercorner].Y + "\t " + vertices[(int)HighDetailFacePoints.RighteyeInnercorner].Z,
                    vertices[(int)HighDetailFacePoints.RighteyeOutercorner].X + "\t" + vertices[(int)HighDetailFacePoints.RighteyeOutercorner].Y + "\t" + vertices[(int)HighDetailFacePoints.RighteyeOutercorner].Z,
                    vertices[(int)HighDetailFacePoints.NoseTip].X + "\t" + vertices[(int)HighDetailFacePoints.NoseTip].Y + "\t" + vertices[(int)HighDetailFacePoints.NoseTip].Z,
                    vertices[(int)HighDetailFacePoints.MouthLeftcorner].X + "\t" + vertices[(int)HighDetailFacePoints.MouthLeftcorner].Y + "\t" + vertices[(int)HighDetailFacePoints.MouthLeftcorner].Z,
                    vertices[(int)HighDetailFacePoints.MouthRightcorner].X + "\t" + vertices[(int)HighDetailFacePoints.MouthRightcorner].Y + "\t" + vertices[(int)HighDetailFacePoints.MouthRightcorner].Z,
                    vertices[(int)HighDetailFacePoints.ChinCenter].X + "\t" + vertices[(int)HighDetailFacePoints.ChinCenter].Y + "\t" + vertices[(int)HighDetailFacePoints.ChinCenter].Z
                };
                if (onCapture == true)
                    File.AppendAllLines(AppDomain.CurrentDomain.BaseDirectory + @"\signData\" + fileName + "_face.txt", lines);

                if (vertices.Count > 0)
                {
                    facePoint = new DepthSpacePoint[vertices.Count];
                    for (int index = 0; index < vertices.Count; index++)
                    {
                        facePoint[index] = kinectSensor.CoordinateMapper.MapCameraPointToDepthSpace(vertices[index]);
                    }
                }
            }
        }

        /// <summary>
        /// Returns the length of a vector from origin
        /// </summary>
        /// <param name="point">Point in space to find it's distance from origin</param>
        /// <returns>Distance from origin</returns>
        private static double VectorLength(CameraSpacePoint point)
        {
            var result = Math.Pow(point.X, 2) + Math.Pow(point.Y, 2) + Math.Pow(point.Z, 2);

            result = Math.Sqrt(result);

            return result;
        }

        /// <summary>
        /// Finds the closest body from the sensor if any
        /// </summary>
        /// <param name="bodyFrame">A body frame</param>
        /// <returns>Closest body, null of none</returns>
        private static Body FindClosestBody(BodyFrame bodyFrame)
        {
            Body result = null;
            double closestBodyDistance = double.MaxValue;

            Body[] bodies = new Body[bodyFrame.BodyCount];
            bodyFrame.GetAndRefreshBodyData(bodies);

            foreach (var body in bodies)
            {
                if (body.IsTracked)
                {
                    var currentLocation = body.Joints[JointType.SpineBase].Position;

                    var currentDistance = VectorLength(currentLocation);

                    if (result == null || currentDistance < closestBodyDistance)
                    {
                        result = body;
                        closestBodyDistance = currentDistance;
                    }
                }
            }

            return result;
        }

        /// <summary>
        /// Find if there is a body tracked with the given trackingId
        /// </summary>
        /// <param name="bodyFrame">A body frame</param>
        /// <param name="trackingId">The tracking Id</param>
        /// <returns>The body object, null of none</returns>
        private static Body FindBodyWithTrackingId(BodyFrame bodyFrame, ulong trackingId)
        {
            Body result = null;

            Body[] bodies = new Body[bodyFrame.BodyCount];
            bodyFrame.GetAndRefreshBodyData(bodies);

            foreach (var body in bodies)
            {
                if (body.IsTracked)
                {
                    if (body.TrackingId == trackingId)
                    {
                        result = body;
                        break;
                    }
                }
            }

            return result;
        }
         
        private void OnTimedEvent(object sender, EventArgs e)
        {
            showTime.Content = stopWatch.ElapsedMilliseconds / 60000 + ":" + (stopWatch.ElapsedMilliseconds % 60000) / 1000 + ":" + stopWatch.ElapsedMilliseconds % 100;
        }
    }
}