using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Net.Sockets;
using System.Diagnostics;
using System.Net;
using Newtonsoft.Json;

namespace GDA_Server_Form
{
    public partial class ServerForm : Form
    {
        //socket declarations
        Socket listener = null;
        Socket picoClient = null;
        Socket appClient = null;

        public ServerForm()
        {
            InitializeComponent();
        }


        private void Form1_Load(object sender, EventArgs e)
        {
            //attempt to connect to pico and app
            StartServer();
            AcceptGDAClient();
        }

        private void StartServer()
        {
            try
            {
                // Create the listening socket with TCP stream
                listener = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
                Trace.WriteLine("listener Socket Created.");

                // Bind the listener socket to any available IP address and port 1666
                listener.Bind(new IPEndPoint(IPAddress.Any, 1666));
                Trace.WriteLine("Listener Binded.");

                // Start listening for incoming connections
                listener.Listen(0);
                Trace.WriteLine("Listening...");

                // Begin accepting the client connection asynchronously
                AcceptPicoClient();
            }
            catch (Exception ex)
            {
                // Log any exceptions that occur during setup
                Trace.WriteLine($"Something went wrong when listening: {ex.Message}");

                // Clean up and reset listener socket if an error occurs
                listener?.Close();
                listener = null;
            }
        }

        private async void AcceptPicoClient()
        {
            try
            {
                //attempt pico client connection asynchronously
                picoClient = await listener.AcceptAsync();

                //close listener
                listener.Close();
                listener = null;
                Trace.WriteLine("Listener Dropped and Closed.");
            }
            catch (Exception ex)
            {
                Trace.WriteLine($"Something went wrong when trying to accept the Pico Connection: {ex.Message}");
                picoClient = null;
                return;
            }
        }

        private async void AcceptGDAClient()
        {
            try
            {
                //attempt connection to GDA Client
                Trace.WriteLine("Attempting Server to GDA Connection...");
                appClient = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
                await appClient.ConnectAsync("localhost", 1666);
            }
            catch (Exception ex)
            {
                Trace.WriteLine($"Server to GDA Connection Failed: {ex.Message}");
                appClient = null;
                return;
            }

            //on success
            Trace.WriteLine("Server to GDA Connection Successful.");
        }

    }
}
