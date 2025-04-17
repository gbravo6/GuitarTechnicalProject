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
using Microsoft.Data.SqlClient;

namespace GuitarDesktopApp
{
    /// <summary>
    /// Interaction logic for Login.xaml
    /// </summary>
    public partial class Login : Page
    {
        //fields and members
        MainWindow mainWin;


        public SqlConnection sqlConnection;

        public Login(MainWindow mainWindow, SqlConnection sqlConn)
        {
            InitializeComponent();
            mainWin = mainWindow;
            sqlConnection = sqlConn;
        }

        private bool Add_User(string user, string pass, SqlConnection con)
        {
            string query = $"INSERT into Users(username, password)" +
                            $"VALUES('{user}', '{pass}')";
            try
            {
                using (SqlCommand cmd = new SqlCommand(query, con))
                {
                    Console.WriteLine("Inserting...");
                    int rowsAffected = cmd.ExecuteNonQuery();
                    if (rowsAffected > 0)
                    {
                        // insert successful
                        Console.WriteLine("INSERT Successful");
                    }
                    else
                    {
                        // No records affected
                        Console.WriteLine("No records added");
                    }
                }
            }
            catch (SqlException ex)
            {
                Console.WriteLine(ex.Message);
                UI_Status.Content = "Unable to add user. Database Error.";
                return false;
            }

            //on ok 
            return true;
        }
        private bool LoginAttempt(string user, string pass, SqlConnection con)
        {
            string query = $"SELECT u.username, u.password " +
                           $"FROM Users u " +
                           $"WHERE u.username = '{user}' " +
                           $"AND u.password = '{pass}' ";
            try
            {
                using (SqlCommand cmd = new SqlCommand(query, con))
                {
                    //create sqldatareader object to store your retrieve data
                    using (SqlDataReader reader = cmd.ExecuteReader())
                    {
                        while (reader.Read())
                        {
                            // Access data using reader["ColumnName] or reader.Getxxx()
                            //Console.WriteLine($"{reader["username"]}");
                            return user == (string)reader["username"];
                        }
                    }
                }
            }
            catch (SqlException ex)
            {
                Console.WriteLine($"Unsucessful Login : {ex.Message}");
            }
            return false;
        }

        private void Button_Click(object sender, RoutedEventArgs e)
        {
            //when login is clicked attempt to login using typed in credentials
            if (LoginAttempt(Username_Box.Text, Password_Box.Password, sqlConnection))
            {
                Trace.WriteLine("Successfull Login");
                UI_Status.Content = "Success";
                NavigationService.Navigate(new Page1(mainWin));
            }
            else
            {
                UI_Status.Content = "Invalid Login.";
            }
        }

        private void Button_Click_1(object sender, RoutedEventArgs e)
        {
            //attempt to add user 

            if (Username_Box.Text != "" && Password_Box.Password != "" &&
                Add_User(Username_Box.Text, Password_Box.Password, sqlConnection))
            {
                UI_Status.Content = "User has been added.";
            }
            else if (Username_Box.Text == "" || Password_Box.Password == "")
            {
                UI_Status.Content = "Username and/or Password cannot be empty!";
            }
            else
            {
                UI_Status.Content = "Could not add user to database.";
            }
        }
    }
}
