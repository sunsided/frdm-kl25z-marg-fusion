using System;
using System.Drawing;
using System.Globalization;
using System.IO.Ports;
using System.Windows.Forms;
using OpenTK;
using OpenTK.Graphics;
using OpenTK.Input;
using ClearBufferMask = OpenTK.Graphics.OpenGL.ClearBufferMask;
using EnableCap = OpenTK.Graphics.OpenGL.EnableCap;
using GetPName = OpenTK.Graphics.OpenGL.GetPName;
using GL = OpenTK.Graphics.OpenGL.GL;
using MatrixMode = OpenTK.Graphics.OpenGL.MatrixMode;

namespace WindowsFormsApplication1
{
    static class Program
    {
        private static readonly ProtocolDecoder Decoder = new ProtocolDecoder();

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
            using (var game = new GameWindow())
            {
                // create the rotation quaternion
                var rotation = new Quaternion(0, 0, 0, 1);

                // the normalization quaternion
                bool bootstrapped = false;
                var unrotate = new Quaternion(0, 0, 0, 1);

                // attach decoder handler
                Decoder.DataReady += (sender, args) =>
                                     {
                                         var data = args.Data;

                                         // check for quaternion data
                                         if (data[0] == 43)
                                         {
                                             float w = BitConverter.ToInt32(data, 1)/65535.0f;
                                             float x = BitConverter.ToInt32(data, 5) / 65535.0f;
                                             float y = BitConverter.ToInt32(data, 9) / 65535.0f;
                                             float z = BitConverter.ToInt32(data, 13) / 65535.0f;

                                             // flip axes from N-E-U to N-W-U
                                             var quat = new Quaternion(x, y, z, w);

                                             // invert quaternion for cake and profit
                                             if (!bootstrapped)
                                             {
                                                 bootstrapped = true;

                                                 unrotate = quat;
                                                 unrotate = Quaternion.Invert(unrotate);
                                             }

                                             // normalize
                                             //quat = unrotate * quat;

                                             // store for rendering
                                             rotation = quat;
                                         }
                                         else if (data[0] == 44)
                                         {
                                             float w = BitConverter.ToInt32(data, 1) / 65535.0f;
                                             float x = BitConverter.ToInt32(data, 5) / 65535.0f;
                                             float y = BitConverter.ToInt32(data, 9) / 65535.0f;
                                             float z = BitConverter.ToInt32(data, 13) / 65535.0f;

                                             double roll = BitConverter.ToInt32(data, 17) / 65535.0 * 180 / Math.PI;
                                             double pitch = BitConverter.ToInt32(data, 21) / 65535.0 * 180 / Math.PI;
                                             double yaw = BitConverter.ToInt32(data, 25) / 65535.0 * 180 / Math.PI;

                                             //Console.WriteLine("roll: {0:##0.00}, pitch: {1:##0.00}, yaw: {2:##0.00}", roll, pitch, yaw);
                                             game.Title = String.Format("roll: {0:##0.00}, pitch: {1:##0.00}, yaw: {2:##0.00}", roll, pitch, yaw);

                                             // flip axes from N-E-U to N-W-U
                                             var quat = new Quaternion(x, y, z, w);

                                             /*
                                             Matrix3 quatMatrix = Matrix3.CreateFromQuaternion(quat);

                                             Matrix3 transform = new Matrix3(1, 0, 0,
                                                                             0, 1, 0,
                                                                             0, 0, 1);

                                             Matrix3 transformed = transform * quatMatrix * transform;

                                             quat = Quaternion.FromMatrix(transformed);
                                              */
                                             

                                             // invert quaternion for cake and profit
                                             if (!bootstrapped)
                                             {
                                                 bootstrapped = true;

                                                 unrotate = Quaternion.Invert(quat);
                                             }

                                             // normalize
                                             quat = unrotate * quat;

                                             // store for rendering
                                             rotation = quat;
                                         }
                                     };

                // attach data receive event
                port.DataReceived += port_DataReceived;

                // begin communication
                port.Open();

                // create projection matrix
                var projection = Matrix4.CreatePerspectiveFieldOfView(MathHelper.PiOver3, (float)game.Width / (float)game.Height, 0.001f, 5000);

                // create look-at matrix
                var lookAt = Matrix4.LookAt(new Vector3(-1.5f, 0.5f, 0.5f), Vector3.Zero, Vector3.UnitY);

                // change OpenGL coordinate system to right-handed coordinate system
                lookAt = OpenGLToTextbook(lookAt);

                game.Load += (sender, e) =>
                {
                    // setup settings, load textures, sounds
                    game.VSync = VSyncMode.On;
                    game.Title = String.Format("Orientation Estimation ({0}, {1} bps)", port.PortName, port.BaudRate);
                };

                game.Resize += (sender, e) =>
                {
                    GL.Viewport(0, 0, game.Width, game.Height);
                };

                game.UpdateFrame += (sender, e) =>
                {
                    // add game logic, input handling
                    if (game.Keyboard[Key.Escape])
                    {
                        game.Exit();
                    }
                    else if (game.Keyboard[Key.R])
                    {
                        bootstrapped = false;
                    }
                };
                
                game.RenderFrame += (sender, e) =>
                {
                    // render graphics
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
                    DrawCoordinateSystem(4);

                    // transform to axis-angle
                    Vector3 axis;
                    float angle;
                    rotation.ToAxisAngle(out axis, out angle);

                    angle = (float) (angle*180.0/Math.PI);

                    // apply rotation
                    GL.Rotate(angle, axis);

                    // draw arrow
                    DrawArrow(Color.Crimson, Color.DarkRed);
                    GL.Translate(0, 0, -0.04f);
                    DrawArrow(Color.Gray, Color.SlateGray);

                    // draw the coordinate system
                    DrawCoordinateSystem(1);

                    game.SwapBuffers();
                };

                // Run the game at 60 updates per second
                game.Run(60.0);
            }
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
