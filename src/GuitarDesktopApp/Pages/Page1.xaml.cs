using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using GuitarDesktopApp.Pages;
using System.Net.Sockets;
using System.Net;

namespace GuitarDesktopApp
{
    /// <summary>
    /// Interaction logic for Page1.xaml
    /// </summary>
    public partial class Page1 : Page
    {
        //properties
        //fields 
        public MainWindow mainWin;
        
        public Page1(MainWindow mainWindow)
        {
            InitializeComponent();
            DataContext = mainWindow;
            mainWin = mainWindow;
        }

        private void UI_Chord_Btn_Click(object sender, RoutedEventArgs e)
        {
            NavigationService.Navigate(new Chords(mainWin));
        }

        private async void UI_Connect_Btn_Click(object sender, RoutedEventArgs e)
        {
            Trace.WriteLine("Connect button clicked");
            StartServer();

            ////check if client is null 
            //if (mainWin._Client != null)
            //{
            //    Trace.WriteLine("Already Connected.");
            //    return;
            //}

            ////attempt a socket connection with the pico w
            //try
            //{
            //    //create new socket
            //    mainWin._Client = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            //    await mainWin._Client.ConnectAsync("localhost", 1666);
            //}
            //catch (SocketException se)
            //{
            //    Trace.WriteLine($"Connect:SocketException : {se.Message}");
            //    mainWin._Client = null;
            //    return;
            //}
            //catch (Exception ex)
            //{
            //    Trace.WriteLine($"Connect:Unkown Issue : {ex.Message}");
            //    mainWin._Client = null;
            //    return;
            //}

            ////if all is good 
            //Trace.WriteLine("Connection Successful.");

        }

        private void UI_Scales_Btn_Click(object sender, RoutedEventArgs e)
        {
            NavigationService.Navigate(new Scales());
        }

        private void StartServer()
        {
            try
            {
                // Create the listening socket with TCP stream
                mainWin.Listener = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
                Trace.WriteLine("listener Socket Created.");

                // Bind the listener socket to any available IP address and port 1666
                mainWin.Listener.Bind(new IPEndPoint(IPAddress.Any, 1666));
                Trace.WriteLine("Listener Binded.");

                // Start listening for incoming connections
                mainWin.Listener.Listen(0);
                Trace.WriteLine("Listening...");

                // Begin accepting the client connection asynchronously
                AcceptPicoClient();
            }
            catch (Exception ex)
            {
                // Log any exceptions that occur during setup
                Trace.WriteLine($"Something went wrong when listening: {ex.Message}");

                // Clean up and reset listener socket if an error occurs
                mainWin.Listener?.Close();
                mainWin.Listener = null;
            }
        }

        private async void AcceptPicoClient()
        {
            try
            {
                //attempt pico client connection asynchronously
                mainWin.pClient = await mainWin.Listener.AcceptAsync();

            }
            catch (Exception ex)
            {
                Trace.WriteLine($"Something went wrong when trying to accept the Pico Connection: {ex.Message}");
                mainWin.pClient = null;
                return;
            }

            //close listener
            mainWin.Listener.Close();
            mainWin.Listener = null;
            Trace.WriteLine("Connection Succesful. Listener Dropped and Closed.");

        }
    }
}
