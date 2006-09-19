	//--------------------------------------------------------------//
        // Constant(s)
        //--------------------------------------------------------------//

        float4 color : Color;
        
        float4x4 transform : WorldViewProjection;

        //--------------------------------------------------------------//
        // Vertex shader(s)
        //--------------------------------------------------------------//

        void FinalVS( inout float4 Position : POSITION0)
        {
	        // Calculate the output position.
	        //float4 tempPos = mul( Position, world );
	        Position = mul( Position, transform);
        }

        //--------------------------------------------------------------//
        // Pixel shader(s)
        //--------------------------------------------------------------//

        void FinalPS(	out float4 Color : COLOR0 )
        {
	        // Set the output color.
	        Color = color;
        }


        //--------------------------------------------------------------//
        // Technique(s)
        //--------------------------------------------------------------//

        technique Final
        {
	        pass Pass_0
	        {
		        VertexShader = compile vs_1_1 FinalVS();
		        PixelShader = compile ps_2_0 FinalPS();
	        }         
        }
