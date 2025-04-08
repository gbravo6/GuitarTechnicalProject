using System.Diagnostics;
using System.Net.Sockets;
using System.Text;
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
using InTheHand.Net.Bluetooth;
using InTheHand.Net.Sockets;

namespace GuitarDesktopApp
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        //fields and members 
        Socket client = null;
        public Socket _Client { get { return client; } set { client = value; } }

        Page1 main;
        public Page1 _main { get { return main; } set { main = value; } }
        Chords chords;
        public Chords _chords { get { return chords; } set { chords = value; } }
        Scales scales;

        public MainWindow()
        {
            InitializeComponent();
            this.Loaded += MainWindow_Loaded;

        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            //pages
            _main = new Page1(this);
            _chords = new Chords(this);
            mainFrame.Content = main;
        }
    }
}