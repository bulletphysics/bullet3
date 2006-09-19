using System;
using System.Collections.Generic;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace Physics
{
    /// <summary>
    /// An abstract class for a bounding volume.
    /// A bounding volume has a transformation matrix that determines the orientation of the volume.
    /// Futhermore it has an intersect funtion that checks whether this bounding volume intersects with an other bounding volume.
    /// There is also a draw method by which the bounding volume is drawn.
    /// </summary>
    public abstract class BoundingVolume
    {
        #region Field: graphicsDevice

        /// <summary>
        /// The graphicsDevice of this volume.
        /// </summary>
        protected GraphicsDevice graphicsDevice;

        #endregion

        #region Field: vertexBuffer

        /// <summary>
        /// The vertex buffer of this volume.
        /// </summary>
        protected VertexBuffer vertexbuffer;

        #endregion

        #region Field: indexBuffer

        /// <summary>
        /// The index buffer of this volume.
        /// </summary>
        protected IndexBuffer indexBuffer;

        #endregion

        #region Field: numberOfPrimitives

        /// <summary>
        /// The number of primitives of this volume.
        /// </summary>
        protected int numberOfPrimitive;

        #endregion

        #region Field: numberOfVertices

        /// <summary>
        /// The number of vertices of this volume.
        /// </summary>
        protected int numberOfVertices;

        #endregion


        #region Property: TransformationMatrix
        
        /// <summary>
        /// The transformation matrix of this bounding volume.
        /// </summary>
        protected Matrix transformationMatrix;

        /// <summary>
        /// The transformation matrix of this bounding volume.
        /// </summary>
        public Matrix TransformationMatrix
        {
            get
            {
                return this.transformationMatrix;
            }
        }

        #endregion

        #region Property: Color
        
        /// <summary>
        /// The color of this bounding volume.
        /// </summary>
        private Color color;

        /// <summary>
        /// The color of this bounding volume.
        /// </summary>
        public Color Color
        {
            get
            {
                return color;
            }
            set
            {
                color = value;
            }
        }

        #endregion


        #region Constructor: BoundingVolume(Color color, GraphicsDevice graphicsDevice)

        /// <summary>
        /// Create a new bounding volume.
        /// </summary>
        /// <param name="color">The color of this volume.</param>
        /// <param name="graphicsDevice">The device used to render.</param>
        public BoundingVolume(Color color, GraphicsDevice graphicsDevice)
        {
            this.color = color;
            this.transformationMatrix = Matrix.Identity;
            this.graphicsDevice = graphicsDevice;
        } 

        #endregion


        #region Method: Intersects(BoundingVolume volume)

        /// <summary>
        /// This method must be implemented.
        /// It checks whether this bounding volume intersects with the given bounding volume.
        /// </summary>
        /// <param name="volume">The volume to check with.</param>
        /// <returns>True if the bounding volumes intersect, false otherwise.</returns>
        public abstract bool Intersects(BoundingVolume volume); 

        #endregion

        #region Method: Draw()

        /// <summary>
        /// Draw this bounding volume to the screen.
        /// </summary>
        public void Draw()
        {
            this.graphicsDevice.RenderState.CullMode = CullMode.None;
            this.graphicsDevice.Indices = this.indexBuffer;
            this.graphicsDevice.VertexDeclaration = new VertexDeclaration(graphicsDevice, VertexPositionColor.VertexElements);
            this.graphicsDevice.Vertices[0].SetSource(this.vertexbuffer, 0, VertexPositionColor.SizeInBytes);
            this.graphicsDevice.DrawIndexedPrimitives(PrimitiveType.TriangleList, 0, 0, this.numberOfVertices, 0, this.numberOfPrimitive);
        } 

        #endregion

        #region Method: TranslateWorld(Vector3 translation)

        /// <summary>
        /// Translate this bounding volume relative to the world axis.
        /// </summary>
        /// <param name="translation">The translation to make.</param>
        public void TranslateWorld(Vector3 translation)
        {
            this.transformationMatrix *= Matrix.CreateTranslation(translation);
        } 

        #endregion
    }
}
