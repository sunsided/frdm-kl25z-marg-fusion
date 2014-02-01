using System;
using System.Drawing;
using System.IO.Ports;
using System.Windows.Forms;
using OpenTK;
using OpenTK.Graphics;
using OpenTK.Input;
using ClearBufferMask = OpenTK.Graphics.OpenGL.ClearBufferMask;
using EnableCap = OpenTK.Graphics.OpenGL.EnableCap;
using GL = OpenTK.Graphics.OpenGL.GL;
using MatrixMode = OpenTK.Graphics.OpenGL.MatrixMode;

namespace WindowsFormsApplication1
{
    static class Program
    {
        /// <summary>
        /// The decoder
        /// </summary>
        private static readonly ProtocolDecoder Decoder = new ProtocolDecoder();

        /// <summary>
        /// The default mode
        /// </summary>
        private const byte DefaultMode = 44;

        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);

            // bring up the port selection dialog
            var portSelection = new PortSelection();
            var result = portSelection.ShowDialog();
            if (result != DialogResult.OK)
            {
                return;
            }

            // fetch the port name
            string portName = portSelection.SelectedPort;

            // Create the serial port with basic settings
            using (var port = new SerialPort(portName, 115200, Parity.None, 8, StopBits.One))
            using (var game = new GameWindow(800, 600, new GraphicsMode(new ColorFormat(32), 24, 0, 4)))
            using (var dump = new DataDump())
            {
                // create the helper window
                var handle = game.WindowInfo.Handle;
                var window = Control.FromHandle(handle);
                dump.SetMode(DefaultMode);
                dump.Show(window);

                game.WindowStateChanged += (sender, args) =>
                                           {
                                               if (game.WindowState == WindowState.Normal)
                                               {
                                                   dump.BringToFront();
                                               }
                                           };

                // last selected mode
                byte lastMode = 255;

                // wire the mode change event
                dump.SelectedModeChanged += (sender, args) =>
                                            {
                                                port.BaseStream.WriteByte(dump.SelectedMode);
                                            };

                // create the rotation quaternion
                var rotation = new Quaternion(0, 0, 0, 1);
                bool hasOrientation = false;

                // the normalization quaternion
                bool bootstrapped = false;
                bool bootstrappingEnabled = false;
                var unrotate = new Quaternion(0, 0, 0, 1);

                // the keyboard states
                KeyboardState previousState = Keyboard.GetState();

                // coordinate systems
                bool primary = false;
                bool secondary = false;

                // to store the default title
                string defaultTitle = String.Empty;

                // attach decoder handler
                Decoder.DataReady += (sender, args) =>
                                     {
                                         var data = args.Data;
                                         string newTitle = defaultTitle;
                                         hasOrientation = false;

                                         var mode = data[0];
                                         if (mode != lastMode)
                                         {
                                             dump.ClearDumps();
                                             lastMode = mode;
                                             bootstrapped = false;
                                         }

                                         // check for raw sensor data
                                         if (mode == 0)
                                         {
                                             float ax = BitConverter.ToInt32(data, 1) / 65535.0f;
                                             float ay = BitConverter.ToInt32(data, 5) / 65535.0f;
                                             float az = BitConverter.ToInt32(data, 9) / 65535.0f;

                                             float mx = BitConverter.ToInt32(data, 13) / 65535.0f;
                                             float my = BitConverter.ToInt32(data, 17) / 65535.0f;
                                             float mz = BitConverter.ToInt32(data, 21) / 65535.0f;

                                             // pipe to dump window
                                             dump.SetAccelerometer(ax, ay, az);
                                             dump.SetMagnetometer(mx, my, mz);

                                             // use TRIAD for orientation
                                             var norm = (float)Math.Sqrt(ax*ax + ay*ay + az*az);
                                             ax /= -norm;
                                             ay /= -norm;
                                             az /= -norm;
                                             norm = (float)Math.Sqrt(ax * ax + ay * ay + az * az);

                                             norm = (float) Math.Sqrt(mx*mx + my*my + mz*mz);
                                             mx /= norm;
                                             my /= norm;
                                             mz /= norm;
                                             norm = (float)Math.Sqrt(mx * mx + my * my + mz * mz);

                                             // cross to create right axis
                                             float nx;
                                             float ny;
                                             float nz;
                                             Cross(ax, ay, az, mx, my, mz, out nx, out ny, out nz);

                                             norm = (float) Math.Sqrt(nx*nx + ny*ny + nz*nz);
                                             nx /= norm;
                                             ny /= norm;
                                             nz /= norm;
                                             norm = (float)Math.Sqrt(nx * nx + ny * ny + nz * nz);

                                             // cross to create forward axis
                                             Cross(nx, ny, nz, ax, ay, az, out mx, out my, out mz);
                                             norm = (float) Math.Sqrt(mx*mx + my*my + mz*mz);

                                             // recreate up axis
                                             Cross(mx, my, mz, nx, ny, nz, out ax, out ay, out az);
                                             norm = (float) Math.Sqrt(ax*ax + ay*ay + az*az);

                                             // create DCM
                                             var dcm = new Matrix3(
                                                                 mx, my, mz,
                                                                 nx, ny, nz,
                                                                 ax, ay, az
                                                                 );

                                             hasOrientation = true;
                                             var quat = Quaternion.FromMatrix(dcm);

                                             // invert quaternion for cake and profit
                                             if (!bootstrapped)
                                             {
                                                 bootstrapped = true;
                                                 unrotate = Quaternion.Invert(quat);
                                             }

                                             // normalize
                                             if (bootstrappingEnabled)
                                             {
                                                 quat = unrotate*quat;
                                             }

                                             // store for rendering
                                             rotation = quat;
                                         }
                                         // check for angle data
                                         else if (mode == 42)
                                         {
                                             hasOrientation = false;
                                             double roll = BitConverter.ToInt32(data, 1) / 65535.0 * 180 / Math.PI;
                                             double pitch = BitConverter.ToInt32(data, 5) / 65535.0 * 180 / Math.PI;
                                             double yaw = BitConverter.ToInt32(data, 9) / 65535.0 * 180 / Math.PI;

                                             //Console.WriteLine("roll: {0:##0.00}, pitch: {1:##0.00}, yaw: {2:##0.00}", roll, pitch, yaw);
                                             newTitle = String.Format("roll: {0:##0.00}, pitch: {1:##0.00}, yaw: {2:##0.00}", roll, pitch, yaw);
                                             
                                             // pipe to dump window
                                             dump.SetAngles(roll, pitch, yaw);
                                         }
                                         // check for quaternion data
                                         else if (mode == 43)
                                         {
                                             hasOrientation = true;
                                             float w = BitConverter.ToInt32(data, 1)/65535.0f;
                                             float x = BitConverter.ToInt32(data, 5) / 65535.0f;
                                             float y = BitConverter.ToInt32(data, 9) / 65535.0f;
                                             float z = BitConverter.ToInt32(data, 13) / 65535.0f;

                                             // compose quaternion
                                             var quat = new Quaternion(x, y, z, w);

                                             // invert quaternion for cake and profit
                                             if (!bootstrapped)
                                             {
                                                 bootstrapped = true;
                                                 unrotate = Quaternion.Invert(quat);
                                             }

                                             // normalize
                                             if (bootstrappingEnabled)
                                             {
                                                 quat = unrotate*quat;
                                             }

                                             // store for rendering
                                             rotation = quat;

                                             // pipe to dump window
                                             dump.SetQuaternion(w, x, y, z);
                                         }
                                         else if (mode == 44)
                                         {
                                             hasOrientation = true;
                                             float w = BitConverter.ToInt32(data, 1) / 65535.0f;
                                             float x = BitConverter.ToInt32(data, 5) / 65535.0f;
                                             float y = BitConverter.ToInt32(data, 9) / 65535.0f;
                                             float z = BitConverter.ToInt32(data, 13) / 65535.0f;

                                             double roll = BitConverter.ToInt32(data, 17) / 65535.0 * 180 / Math.PI;
                                             double pitch = BitConverter.ToInt32(data, 21) / 65535.0 * 180 / Math.PI;
                                             double yaw = BitConverter.ToInt32(data, 25) / 65535.0 * 180 / Math.PI;

                                             //Console.WriteLine("roll: {0:##0.00}, pitch: {1:##0.00}, yaw: {2:##0.00}", roll, pitch, yaw);
                                             newTitle = String.Format("roll: {0:##0.00}, pitch: {1:##0.00}, yaw: {2:##0.00}", roll, pitch, yaw);

                                             // compose quaternion
                                             var quat = new Quaternion(x, y, z, w);

                                            /*
                                             var quatMatrix = Matrix3.CreateFromQuaternion(quat);
                                             var transform = new Matrix3(1, 0, 0,
                                                                         0, 1, 0,
                                                                         0, 0, 1);

                                             Matrix3 transformed = transform * quatMatrix * Matrix3.Transpose(transform);
                                             quat = Quaternion.FromMatrix(transformed);
                                              */
                                             

                                             // invert quaternion for cake and profit
                                             if (!bootstrapped)
                                             {
                                                 bootstrapped = true;
                                                 unrotate = Quaternion.Invert(quat);
                                             }

                                             // normalize
                                             if (bootstrappingEnabled)
                                             {
                                                 quat = unrotate*quat;
                                             }

                                             // store for rendering
                                             rotation = quat;

                                             // pipe to dump window
                                             dump.SetQuaternion(w, x, y, z);
                                             dump.SetAngles(roll, pitch, yaw);
                                         }

                                         game.Title = newTitle;
                                     };

                // attach data receive event
                port.DataReceived += port_DataReceived;

                // begin communication
                port.Open();
                port.BaseStream.WriteByte(DefaultMode);

                // create projection matrix
                var projection = Matrix4.CreatePerspectiveFieldOfView(MathHelper.PiOver3, (float)game.Width / (float)game.Height, 0.001f, 5000);

                // create look-at matrix
                //var lookAt = Matrix4.LookAt(new Vector3(-1.5f, 0.5f, 0.5f), Vector3.Zero, Vector3.UnitY);
                var lookAt = Matrix4.LookAt(new Vector3(-1.5f, 0.5f, 0.0f), Vector3.Zero, Vector3.UnitY);

                // change OpenGL coordinate system to right-handed coordinate system
                lookAt = OpenGLToTextbook(lookAt);

                game.Load += (sender, e) =>
                {
                    // setup settings, load textures, sounds
                    game.VSync = VSyncMode.On;

                    defaultTitle = String.Format("Orientation Estimation ({0}, {1} bps)", port.PortName, port.BaudRate);
                    game.Title = defaultTitle;

                };

                game.Resize += (sender, e) =>
                {
                    GL.Viewport(0, 0, game.Width, game.Height);
                };

                game.UpdateFrame += (sender, e) =>
                                    {
                                        var currentState = Keyboard.GetState();

                                        // add game logic, input handling
                                        if (currentState.IsKeyDown(Key.Escape) && !previousState.IsKeyDown(Key.Escape))
                                        {
                                            game.Exit();
                                        }
                                        else if (currentState.IsKeyDown(Key.R) && !previousState.IsKeyDown(Key.R))
                                        {
                                            bootstrappingEnabled = !bootstrappingEnabled;
                                            if (bootstrappingEnabled)
                                            {
                                                bootstrapped = false;
                                            }
                                        }
                                        else if (currentState.IsKeyDown(Key.Number1) &&
                                                 !previousState.IsKeyDown(Key.Number1))
                                        {
                                            primary = !primary;
                                        }
                                        else if (currentState.IsKeyDown(Key.Number2) &&
                                                 !previousState.IsKeyDown(Key.Number2))
                                        {
                                            secondary = !secondary;
                                        }
                                        else if (currentState.IsKeyDown(Key.Space) &&
                                                 !previousState.IsKeyDown(Key.Space))
                                        {
                                            dump.BringToFront();
                                        }

                                        previousState = currentState;
                                    };
                
                game.RenderFrame += (sender, e) =>
                {
                    // render graphics
                    GL.ClearColor(Color.Black);
                    if (bootstrappingEnabled)
                    {
                        GL.ClearColor(Color.MidnightBlue);
                    }
                    GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

                    // set options
                    GL.Enable(EnableCap.DepthTest);
                    GL.Disable(EnableCap.CullFace);
                    GL.Enable(EnableCap.Multisample);

                    // set projection matrix
                    GL.MatrixMode(MatrixMode.Projection);
                    GL.LoadMatrix(ref projection);

                    // set model view
                    GL.MatrixMode(MatrixMode.Modelview);
                    GL.LoadMatrix(ref lookAt);

                    // draw the coordinate system
                    if (primary)
                    {
                        DrawCoordinateSystem(4);
                    }

                    // transform to axis-angle
                    Vector3 axis;
                    float angle;
                    rotation.ToAxisAngle(out axis, out angle);

                    angle = (float) (angle*180.0/Math.PI);

                    // apply rotation
                    GL.Rotate(angle, axis);

                    // draw arrow
                    if (hasOrientation)
                    {
                        DrawArrow(Color.Crimson, Color.DarkRed);
                        GL.Translate(0, 0, -0.04f);
                        DrawArrow(Color.Gray, Color.SlateGray);

                        // draw the coordinate system
                        if (secondary)
                        {
                            GL.Translate(0, 0, +0.02f);
                            DrawCoordinateSystem(1);
                        }
                    }

                    game.SwapBuffers();
                };

                // Run the game at 60 updates per second
                game.Run(60.0);
            }
        }

        private static void Cross(float ax, float ay, float az, float bx, float by, float bz, out float cx, out float cy, out float cz)
        {
            cx = ay * bz - az * by;
            cy = az * bx - ax * bz;
            cz = ax * by - ay * bx;
        }

        private static Matrix4 OpenGLToTextbook(Matrix4 lookAt)
        {
            var transform = new Matrix4(1, 0, 0, 0,
                0, 0, -1, 0,
                0, 1, 0, 0,
                0, 0, 0, 1);

            lookAt = transform*lookAt;
            return lookAt;
        }

        /// <summary>
        /// Handles the DataReceived event of the port control.
        /// </summary>
        /// <param name="sender">The source of the event.</param>
        /// <param name="e">The <see cref="SerialDataReceivedEventArgs"/> instance containing the event data.</param>
        /// <exception cref="System.NotImplementedException"></exception>
        static void port_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                var port = sender as SerialPort;
                while (port.BytesToRead > 0)
                {
                    var datum = port.ReadByte();
                    Decoder.Decodify(datum);
                }
            }
// ReSharper disable once EmptyGeneralCatchClause
            catch
            {
                
            }
        }

        /// <summary>
        /// Draws the arrow.
        /// </summary>
        /// <param name="a">A.</param>
        /// <param name="b">The b.</param>
        private static void DrawCoordinateSystem(float lineWidth = 1.0f)
        {
            GL.LineWidth(lineWidth);
            GL.Begin(OpenTK.Graphics.OpenGL.BeginMode.Lines);

            GL.Color3(Color.Red);
            GL.Vertex3(0, 0, 0);
            GL.Vertex3(100.0f, 0, 0);

            GL.Color3(Color.Green);
            GL.Vertex3(0, 0, 0);
            GL.Vertex3(0, 100.0f, 0);

            GL.Color3(Color.Blue);
            GL.Vertex3(0, 0, 0);
            GL.Vertex3(0, 0, 100.0f);

            GL.End();
        }     

        /// <summary>
        /// Draws the arrow.
        /// </summary>
        /// <param name="a">A.</param>
        /// <param name="b">The b.</param>
        private static void DrawArrow(Color a, Color b)
        {
            GL.Begin(OpenTK.Graphics.OpenGL.BeginMode.Polygon);
            GL.Color3(a);
            GL.Vertex3(0.75f, 0f, 0.02f);
            GL.Vertex3(0.25f, 0.50f, 0.02f);
            GL.Vertex3(0.25f, 0.25f, 0.02f);
            GL.Vertex3(-0.75f, 0.25f, 0.02f);
            GL.Color3(b);
            GL.Vertex3(-0.75f, -0.25f, 0.02f);
            GL.Vertex3(0.25f, -0.25f, 0.02f);
            GL.Vertex3(0.25f, -0.50f, 0.02f);
            GL.Vertex3(0.75f, 0f, 0.02f);
            GL.End();
        }        
    }
}
