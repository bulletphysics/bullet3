using System;
using System.Collections.Generic;
using System.Text;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework;

namespace Physics
{
    /// <summary>
    /// This bounding volume is represented by a box.
    /// </summary>
    public class BoundingBox : BoundingVolume
    {
        #region Constructor: BoundingBox(float width, float height, float depth, Color color, GraphicsDevice device)

        /// <summary>
        /// Creates a new bounding box.
        /// </summary>
        /// <param name="width">The width of the box.</param>
        /// <param name="height">The height of the box.</param>
        /// <param name="depth">The depth of the box.</param>
        /// <param name="color">The color of the box.</param>
        /// <param name="device">The rendering device.</param>
        public BoundingBox(float width, float height, float depth, Color color, GraphicsDevice device)
            : base(color, device)
        {
            #region Vertex Buffer

            // Define the vertices
            VertexPositionColor[] tempVertices = new VertexPositionColor[8];
            float halfWidth = width / 2f;
            float halfHeight = height / 2f;
            float halfDepth = depth / 2f;

            // Define the 8 corners.
            tempVertices[0] = new VertexPositionColor(new Vector3(-halfWidth, -halfHeight, halfDepth), Color.White);
            tempVertices[1] = new VertexPositionColor(new Vector3(-halfWidth, -halfHeight, -halfDepth), Color.White);
            tempVertices[2] = new VertexPositionColor(new Vector3(halfWidth, -halfHeight, -halfDepth), Color.White);
            tempVertices[3] = new VertexPositionColor(new Vector3(halfWidth, -halfHeight, halfDepth), Color.White);
            tempVertices[4] = new VertexPositionColor(new Vector3(-halfWidth, halfHeight, halfDepth), Color.White);
            tempVertices[5] = new VertexPositionColor(new Vector3(-halfWidth, halfHeight, -halfDepth), Color.White);
            tempVertices[6] = new VertexPositionColor(new Vector3(halfWidth, halfHeight, -halfDepth), Color.White);
            tempVertices[7] = new VertexPositionColor(new Vector3(halfWidth, halfHeight, halfDepth), Color.White);

            // Initialize the vertex buffer.
            this.vertexbuffer = new VertexBuffer(device, typeof(VertexPositionColor), 8, ResourceUsage.None, ResourcePool.Default);

            // Set the vertices.
            this.vertexbuffer.SetData<VertexPositionColor>(tempVertices);

            // Set other info.
            this.numberOfPrimitive = 12;
            this.numberOfVertices = 8;

            #endregion

            #region Index Buffer

            // Define the index buffer.
            int[] tempIndices = new int[12 * 3];

            // Front face.
            tempIndices[0] = 0;
            tempIndices[1] = 4;
            tempIndices[2] = 7;
            tempIndices[3] = 0;
            tempIndices[4] = 7;
            tempIndices[5] = 3;

            // Back face.
            tempIndices[6] = 2;
            tempIndices[7] = 6;
            tempIndices[8] = 5;
            tempIndices[9] = 2;
            tempIndices[10] = 5;
            tempIndices[11] = 1;

            // Left face.
            tempIndices[12] = 1;
            tempIndices[13] = 5;
            tempIndices[14] = 4;
            tempIndices[15] = 1;
            tempIndices[16] = 4;
            tempIndices[17] = 0;

            // Right face.
            tempIndices[18] = 4;
            tempIndices[19] = 7;
            tempIndices[20] = 6;
            tempIndices[21] = 4;
            tempIndices[22] = 6;
            tempIndices[23] = 3;

            // Up face.
            tempIndices[24] = 4;
            tempIndices[25] = 5;
            tempIndices[26] = 6;
            tempIndices[27] = 4;
            tempIndices[28] = 6;
            tempIndices[29] = 7;

            // Down face.
            tempIndices[30] = 0;
            tempIndices[31] = 2;
            tempIndices[32] = 1;
            tempIndices[33] = 0;
            tempIndices[34] = 3;
            tempIndices[35] = 2;

            // Initialize the index buffer.
            this.indexBuffer = new IndexBuffer(device, typeof(int), 12 * 3, ResourceUsage.None, ResourcePool.Default);

            // Set the data.
            this.indexBuffer.SetData<int>(tempIndices);

            #endregion
        } 

        #endregion


        #region Method: Intersects(BoundingVolume volume)

        /// <summary>
        /// It checks whether this bounding box intersects with the given bounding volume.
        /// </summary>
        /// <param name="volume">The volume to check with.</param>
        /// <returns>True if the bounding volumes intersect, false otherwise.</returns>
        public override bool Intersects(BoundingVolume volume)
        {
            // Check with other bounding volumes.
            if (volume is BoundingBox)
            {
                // Intersection code here.
            }

            // By default.
            return false;
        }

        #endregion
    }
}
