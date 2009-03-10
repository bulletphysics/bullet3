using System;
using System.Collections.Generic;
using System.Windows.Forms;

using Scea.BulletPhysics;

namespace bulletapp
{
    static class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {
            Bullet bullet = new Bullet();

            CollisionShapeHandle s_handle = bullet.CreateSphere(1);
            RigidBody body = new RigidBody(1, s_handle);
            float [] position = new float[3];
            position[0] = 1.0f; position[1] = 1.1f; position[2] = 1.2f;
            body.SetPosition(position);
            body.SetOrientation(0.1f, 0.2f, 0.3f);

            bullet.AddRigidBody(body);
            System.Console.WriteLine("Original Position      (" + position[0] + ", " + position[1] + ", " + position[2] + ")");

            float[] new_position = new float[3];
            for (int i = 0; i < 10; i++)
            {
                bullet.StepSimulation(0.1f);
                System.Console.WriteLine("StepSimulation " + 0.1);
                body.GetPosition(new_position);
                System.Console.WriteLine("New Position           (" + new_position[0] + ", " + new_position[1] + ", " + new_position[2] + ")");
            }

            float[] matrix = new float[16];
            body.GetOpenGLMatrix(matrix);
            System.Console.WriteLine("GetOpenGLMatrix        (" + 
                matrix[0] + ", " + matrix[1] + ", " + matrix[2] + ", " + matrix[3] + ", " +
                matrix[4] + ", " + matrix[5] + ", " + matrix[6] + ", " + matrix[7] + ", " +
                matrix[8] + ", " + matrix[9] + ", " + matrix[10] + ", " + matrix[11] + ", " +
                matrix[12] + ", " + matrix[13] + ", " + matrix[14] + ", " + matrix[15] + ")"
                );

            float[] quot = new float[4];
            body.GetOrientation(quot);
            System.Console.WriteLine("GetOrientation         (" + quot[0] + ", " + quot[1] + ", " + quot[2] + ", " + quot[3] + ")");
        }
    }
}