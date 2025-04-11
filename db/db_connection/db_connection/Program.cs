using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Data.SqlClient;
using Azure.Core;
using static System.Windows.Forms.VisualStyles.VisualStyleElement.StartPanel;
using System.Drawing;

namespace db_connection
{
    internal class Program
    {
        static void Main(string[] args)
        {
            /*
             * Define a connection string that contains information about your database server,
             * DB name, authentication
            */
            string connectiongString = "Server=guitargod.database.windows.net;" +
                            "Database=GuitarGodUsersDB;" +
                            "User Id=Charlie;" +
                            "Password=cmpe200549555!;" +
                            "Encrypt=False";
            
            Connect(connectiongString);
        }
        static void Connect(string connString)
        {
            using (SqlConnection conn = new SqlConnection(connString))
            {
                Console.WriteLine("Connecting to server");
                try
                {
                    conn.Open();
                    Console.WriteLine("Connection is open");
                    //Add_User("test", "1234", conn);
                    Login("test_user", "12345", conn);
                    return;
                }
                catch (SqlException ex)
                {
                    Console.WriteLine(ex.Message);
                }
                Console.WriteLine("Failed to connect");
            }
        }
        static void Add_User(string user, string pass, SqlConnection con)
        {
            string query = "INSERT into Users(username, password)"+
                            "VALUES('test_user', '12345')";
            try
            {
                using (SqlCommand cmd = new SqlCommand(query, con))
                {
                    Console.WriteLine("Inserting...");
                    int rowsAffected = cmd.ExecuteNonQuery();
                    if (rowsAffected > 0)
                    {
                        // Delete successful
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
            }
        }
        static bool Login(string user, string pass, SqlConnection con)
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
                Console.WriteLine(ex.Message);
            }
            return false;
        }
    }
}
