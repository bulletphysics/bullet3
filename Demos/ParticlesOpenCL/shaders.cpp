#define STRINGIFY(A) #A
//use most up-to-date AMD Radeon drivers to make point sprites work
//see also http://forums.amd.com/devforum/messageview.cfm?catid=392&threadid=129431
// vertex shader
const char *vertexShader = STRINGIFY(
uniform float pointRadius;  // point size in world space
uniform float pointScale;   // scale to calculate size in pixels
uniform float densityScale;
uniform float densityOffset;
varying vec3 posEye;
void main()
{
    // calculate window-space point size
    posEye = vec3(gl_ModelViewMatrix * vec4(gl_Vertex.xyz, 1.0));
    float dist = length(posEye);
    gl_PointSize = pointRadius * (pointScale / dist);

	gl_TexCoord[0] = gl_MultiTexCoord0;

	gl_Position = gl_ModelViewProjectionMatrix * vec4(gl_Vertex.xyz, 1.0);

    gl_FrontColor = gl_Color;
}
);



// pixel shader for rendering points as shaded spheres
const char *spherePixelShader = STRINGIFY(
uniform float pointRadius;  // point size in world space
varying vec3 posEye;        // position of center in eye space
void main()
{
    const vec3 lightDir = vec3(0.577, 0.577, 0.577);
    const float shininess = 40.0;

    // calculate normal from texture coordinates
    vec3 N;
    N.xy = gl_TexCoord[0].xy*vec2(2.0, -2.0) + vec2(-1.0, 1.0);
    float mag = dot(N.xy, N.xy);
    if (mag > 1.0) discard;   // kill pixels outside circle
    N.z = sqrt(1.0-mag);

    // point on surface of sphere in eye space
    vec3 spherePosEye = posEye + N*pointRadius;

    // calculate lighting
    float diffuse = max(0.0, dot(lightDir, N));
//    gl_FragColor = gl_Color * diffuse;

	vec3 v = normalize(-spherePosEye);
    vec3 h = normalize(lightDir + v);
    float specular = pow(max(0.0, dot(N, h)), shininess);
    gl_FragColor = gl_Color * diffuse + specular;
}
);
