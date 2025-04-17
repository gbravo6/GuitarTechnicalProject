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
using System.Net;
using Microsoft.Data.SqlClient;

namespace GuitarDesktopApp
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        //fields and members 

        Page1 main;
        public Page1 _main { get { return main; } set { main = value; } }

        public UdpClient udpClient { get; set; }

        SqlConnection sqlConnection;
        //connection string for connecting to azure database
        string connectionString = "Server=guitargod.database.windows.net;" +
                "Database=GuitarGodUsersDB;" +
                "User Id=Charlie;" +
                "Password=cmpe200549555!;" +
                "Encrypt=False";

        public MainWindow()
        {
            InitializeComponent();
            this.Loaded += MainWindow_Loaded;
        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            //pages
            _main = new Page1(this);
            Connect(connectionString);
            mainFrame.Content = new Login(this, sqlConnection);
        }
        void Connect(string connString)
        {

            sqlConnection = new SqlConnection(connString);

            Console.WriteLine("Connecting to server");
            try
            {
                sqlConnection.Open();
                Console.WriteLine("Connection is open");
                return;
            }
            catch (SqlException ex)
            {
                Console.WriteLine(ex.Message);
                Console.WriteLine("Failed to connect");
                return;
            }


        }

    }
}