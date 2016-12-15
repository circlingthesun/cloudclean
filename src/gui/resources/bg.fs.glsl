#version 330
 
layout(location = 0) out vec3 fragColor;



const vec2 mouse = vec2(0, 0);
const float time = 1.0;
uniform vec2 resolution;

const vec3 color1 = vec3(0.41, 0.57, 0.71); // 0
const vec3 color2 = vec3(0.31, 0.44, 0.58); // 0.4
const vec3 color3 = vec3(0.06, 0.22, 0.47); // 0.8
const vec3 color4 = vec3(1., 1., 1.);

void main( void )
{

    vec2 uv = gl_FragCoord.xy/(resolution.xy);

    float dist = pow((uv.x - 0.5) * 2, 2) + pow((uv.y - 0.5) * 2, 2);
    dist = dist * 0.40;

//    if(dist < 0.4){
//        fragColor = mix(color1, color2, dist/0.4);
//    } else if (dist > 0.4 && dist < 0.8) {
//        fragColor = mix(color2, color3, (dist-0.4)/0.4);
//    } else {
//        fragColor = mix(color3, color4, (dist-0.8)/0.2);
//    }


    if(dist < 0.4){
        fragColor = mix(color1, color2, dist/0.4);
    } else if (dist > 0.4 && dist < 0.8) {
        fragColor = mix(color2, color3, (dist-0.4)/0.4);
    } else {
        fragColor = color3;
    }




//    vec3 col = vec3(0., 0., 0.);
//    const int itrCount = 10;
//    for (int i = 0; i < itrCount; ++i)
//    {

//            float offset = float(i) / float(itrCount);
//            float t = time + (offset * offset * 2.);

//            vec2 pos=(gl_FragCoord.xy/resolution.xy);
//            pos.y-=0.5;
//            pos.y+=sin(pos.x*9.0+t)*.2*sin(t*.8);
//            float color=1.0-pow(abs(pos.y),0.2);
//            float colora=pow(1.,0.2*abs(pos.y));

//            float rColMod = ((offset * .5) + .5) * colora;
//            float bColMod = (1. - (offset * .5) + .5) * colora;

//            col += vec3(color * rColMod, color, color * bColMod) * (1. / float(itrCount));
//    }
//    col = clamp(col, 0., 1.);

//    fragColor=vec4(col.x, col.y, col.z ,1.0);

}
