// jds - javascript data structures

// ECMAScript module

//npm install https://github.com/PeterTadich/homogeneous-transformations https://github.com/PeterTadich/matrix-computations

/*
generate_graph();
//print_graph();
var edge = graph.edges[1][0];
var myKeys = Object.keys(edge);
console.log(myKeys);

var y = graph.edges[1][0].v;
console.log(y);
//var elm = graph.edges[1][0]['T'][1][1];
//var elm = graph.edges[1][0].T[1][1];
var elm = graph.edges[1][0].T;
console.log(elm);
*/

import * as hlao from 'matrix-computations';
import * as mcht from 'homogeneous-transformations';
//import * as hlao from '../matrix-computations/hlao.mjs';
//import * as mcht from '../homogeneous-transformations/mcht.mjs';

//REF: Skiena Programming Challenges, page 192
var MAXV = 50;     /* maximum number of vertices */
var MAXDEGREE = 10; /* maximum vertex outdegree */

//graph data structure (javascript)
//notes: REF: C:\xampp\htdocs\tutorials\DA\javascriptInfluenceDiagram.js
var graph = {
    edges: [],
    degree: [],
    occupied: [],
    pose: [],
    nvertices: 0,
    nedges: 0,
};

function initialize_graph(){ //REF: Skiena Programming Challenges, page 193 
   graph.nvertices = 1; //one more than the number of edges
   graph.nedges = 0;
   for (var i=1; i<=MAXV; i++){
        graph.edges[i] = [];
        graph.degree[i] = 0;
        graph.occupied[i] = 0;
        graph.pose[i] = "";
    }
}

function insert_edge(x, y, T, frame_data, directed){ //REF: Skiena Programming Challenges, page 193    
    if (graph.degree[x] > MAXDEGREE) console.log("Warning: insertion (" + x + "," + y + ") exceeds max degree");
    //graph.edges[vertex 'x'][vertex 'x' degree]
    graph.edges[x][graph.degree[x]] = {
        v: y,              //vertex y
                           //Denavit-Hartenberg parameters, REF: Robotics Modelling, Planning and Control, Page 63
        T: [
            [T[0][0], T[0][1], T[0][2], T[0][3]],
            [T[1][0], T[1][1], T[1][2], T[1][3]],
            [T[2][0], T[2][1], T[2][2], T[2][3]],
            [T[3][0], T[3][1], T[3][2], T[3][3]]
        ]
    };
    //console.log(graph.pose[x].length);
    if(graph.pose[x].length == 0) graph.pose[x] = frame_data; //title of frame
    else {
        if(graph.pose[x] != frame_data) alert("Frame error: " + graph.pose[x] + " != " + frame_data + " [node " + x + "]");
    }
    graph.occupied[x] = 1;
    graph.occupied[y] = 1;
    graph.degree[x]++;
    if (directed == false){
        //var Tinverse = ;
        //var frame_data = = "Frame z" + (i-1);
        insert_edge(y,x,Tinverse,frame_data,true);
    } else {
        graph.nedges++;
        graph.nvertices++;
    }
}

function print_graph(){ //REF: Skiena Programming Challenges, page 194
    var str_dummy = "";
    
    for (var i=1; i<=MAXV; i++) {
        if(graph.occupied[i]){
            str_dummy = str_dummy + "   N" + i + " [pose: " + graph.pose[i] + "] (degree: " + graph.degree[i] + ") --- ";
            for (var j=0; j<graph.degree[i]; j++){
                str_dummy = str_dummy + graph.edges[i][j].v + "[pose: " + graph.pose[graph.edges[i][j].v] + "] ";
            }
            str_dummy = str_dummy + "\n";
        }
    }
    
    str_dummy = str_dummy + "number of nodes: " + graph.nvertices + "\n";
    str_dummy = str_dummy + "number of edges: " + graph.nedges + "\n";
    
    alert(str_dummy);
}

function generate_graph(){
    //***********************DH Parameters start
    var W = 0.072; var L = 0.072; // mm
    var L1 = L/2.0; var L2 = W/2.0;
    var L3 = 0.029; var L4 = 0.057; var L5 = 0.080;
    
    var piH = Math.PI/2.0;
    
    //body link (at the inertial reference frame)
    var bl = [ //node 1 link 1 (body link) (inertial ref frame {0} to body link {N1})
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ];
    //Leg 1
    var vl1 = [ //node 2 link 2 (virtual link) (node 1 {N1} to node 2 {N2})
        [1.0, 0.0, 0.0, -L2],
        [0.0, 1.0, 0.0, -L1],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ];
    var l1 = [
      //[  ai,  alpha_i,  di,  vi]
        [ -L3,     -piH, 0.0, 0.0], //link 3 (node 3 {N3})
        [ -L4,      0.0, 0.0, 0.0], //link 4 (node 4 {N4})
        [  L5,      0.0, 0.0, piH]  //link 5 (node 5 {N5})
    ];
    //Leg 2
    var vl2 = [ //node 6 link 6 (virtual link) (node 1 {N1} to node 6 {N6})
        [1.0, 0.0, 0.0, -L2],
        [0.0, 1.0, 0.0,  L1],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ];
    var l2 = [
      //[  ai,  alpha_i,  di,  vi]
        [ -L3,     -piH, 0.0, 0.0], //link 7 (node 7 {N7})
        [ -L4,      0.0, 0.0, 0.0], //link 8 (node 8 {N8})
        [  L5,      0.0, 0.0, piH]  //link 9 (node 9 {N9})
    ];
    //Leg 3
    var vl3 = [ //node 10 link 10 (virtual link) (node 1 {N1} to node 10 {N10})
        [1.0, 0.0, 0.0,  L2],
        [0.0, 1.0, 0.0, -L1],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ];
    var l3 = [
      //[  ai,  alpha_i,  di,  vi]
        [  L3,     -piH, 0.0, 0.0], //link 11 (node 11 {N11})
        [  L4,      0.0, 0.0, 0.0], //link 12 (node 12 {N12})
        [  L5,      0.0, 0.0, piH]  //link 13 (node 13 {N13})
    ];
    //Leg 4
    var vl4 = [ //node 14 link 14 (virtual link) (node 1 {N1} to node 14 {N14})
        [1.0, 0.0, 0.0,  L2],
        [0.0, 1.0, 0.0,  L1],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ];
    var l4 = [
      //[  ai,  alpha_i,  di,  vi]
        [  L3,     -piH, 0.0, 0.0], //link 15 (node 15 {N15})
        [  L4,      0.0, 0.0, 0.0], //link 16 (node 16 {N16})
        [  L5,      0.0, 0.0, piH]  //link 17 (node 17 {N17})
    ];
    //***********************DH Parameters end
    
    initialize_graph();
    
    /* 
       N1 [pose: Frame {bl}] (degree: 4) --- 2[pose: Frame {z2}] 6[pose: Frame {z6}] 10[pose: Frame {z10}] 14[pose: Frame {z14}] 
       N2 [pose: Frame {z2}] (degree: 1) --- 3[pose: Frame {z3}] 
       N3 [pose: Frame {z3}] (degree: 1) --- 4[pose: Frame {z4}] 
       N4 [pose: Frame {z4}] (degree: 1) --- 5[pose: ] 
       N5 [pose: ] (degree: 0) --- 
       N6 [pose: Frame {z6}] (degree: 1) --- 7[pose: Frame {z7}] 
       N7 [pose: Frame {z7}] (degree: 1) --- 8[pose: Frame {z8}] 
       N8 [pose: Frame {z8}] (degree: 1) --- 9[pose: ] 
       N9 [pose: ] (degree: 0) --- 
       N10 [pose: Frame {z10}] (degree: 1) --- 11[pose: Frame {z11}] 
       N11 [pose: Frame {z11}] (degree: 1) --- 12[pose: Frame {z12}] 
       N12 [pose: Frame {z12}] (degree: 1) --- 13[pose: ] 
       N13 [pose: ] (degree: 0) --- 
       N14 [pose: Frame {z14}] (degree: 1) --- 15[pose: Frame {z15}] 
       N15 [pose: Frame {z15}] (degree: 1) --- 16[pose: Frame {z16}] 
       N16 [pose: Frame {z16}] (degree: 1) --- 17[pose: ] 
       N17 [pose: ] (degree: 0) --- 
       N50 [pose: Frame {0}] (degree: 1) --- 1[pose: Frame {bl}] 
    number of nodes: 18
    number of edges: 17
    */
    
    //insert:
    var x = 50; //node 50 (inertial frame)
    var y =  1; //node  1 (body frame)
    var frame = "Frame {0}";
    var directed = true;
    insert_edge(x, y, bl, frame, directed);
    //   - {N1} to {N2} (body link to leg 1)
    var x = 1; //node 1
    var y = 2; //node 2
    var frame = "Frame {bl}";
    var directed = true;
    insert_edge(x, y, vl1, frame, directed);
    //   - {N1} to {N6} (body link to leg 2)
    var x = 1; //node 1
    var y = 6; //node 6
    var frame = "Frame {bl}";
    var directed = true;
    insert_edge(x, y, vl2, frame, directed);
    //   - {N1} to {N10} (body link to leg 3)
    var x = 1; //node 1
    var y = 10; //node 10
    var frame = "Frame {bl}";
    var directed = true;
    insert_edge(x, y, vl3, frame, directed);
    //   - {N1} to {N14} (body link to leg 4)
    var x = 1; //node 1
    var y = 14; //node 14
    var frame = "Frame {bl}";
    var directed = true;
    insert_edge(x, y, vl4, frame, directed);
    //   - leg 1 (N2 to N5)
    for(var i=0;i<l1.length;i=i+1){ //node increment
        var x = i+2;
        var y = x+1;
        var T = mcht.Aij(l1[i]); //transform from from {Ai} to {Ai-1}
        //console.log(T);
        var frame = "Frame {z" + x + "}";
        var directed = true;
        insert_edge(x, y, T, frame, directed);
    }
    //   - leg 2 (N6 to N9)
    for(var i=0;i<l2.length;i=i+1){ //node increment
        var x = i+6;
        var y = x+1;
        var T = mcht.Aij(l2[i]); //transform from from {Ai} to {Ai-1}
        //console.log(T);
        var frame = "Frame {z" + x + "}";
        var directed = true;
        insert_edge(x, y, T, frame, directed);
    }
    //   - leg 3 (N10 to N13)
    for(var i=0;i<l3.length;i=i+1){ //node increment
        var x = i+10;
        var y = x+1;
        var T = mcht.Aij(l3[i]); //transform from from {Ai} to {Ai-1}
        //console.log(T);
        var frame = "Frame {z" + x + "}";
        var directed = true;
        insert_edge(x, y, T, frame, directed);
    }
    //   - leg 4 (N14 to N17)
    for(var i=0;i<l4.length;i=i+1){ //node increment
        var x = i+14;
        var y = x+1;
        var T = mcht.Aij(l4[i]); //transform from from {Ai} to {Ai-1}
        //console.log(T);
        var frame = "Frame {z" + x + "}";
        var directed = true;
        insert_edge(x, y, T, frame, directed);
    }
}

/*
init_queue();
//console.log(empty());
enqueue(22);
enqueue(-1.937);
enqueue(6.0);
enqueue(100203.0);
console.log(dequeue());
console.log(dequeue());
console.log(dequeue());
console.log(dequeue());
console.log(dequeue());
*/

var QUEUESIZE = 50;

// ref: Skiena, Programming Challenges, page 29
var queue = {
    q: [], //body of queue
    first: 0, //position of first element
    last: 0, //position of last element
    count: 0 //number of queue elements
};

function init_queue(){
    queue.first = 0;
    queue.last = QUEUESIZE-1;
    queue.count = 0;
}

function enqueue(x){
    if (queue.count >= QUEUESIZE) console.log("Warning: queue overflow enqueue x=",x);
    else {
        queue.last = (queue.last+1) % QUEUESIZE;
        queue.q[ queue.last ] = x;
        queue.count = queue.count + 1;
    }
}

function dequeue(){
    var x;
    if (queue.count <= 0) console.log("Warning: empty queue dequeue.");
    else {
        x = queue.q[ queue.first ];
        queue.first = (queue.first+1) % QUEUESIZE;
        queue.count = queue.count - 1;
    }
    return(x);
}

function empty(){
    if (queue.count <= 0) return (true);
    else return (false);
}

/*
generate_graph();
print_graph();
init_queue();
initialize_search();
console.log(graph.nvertices);
console.log(processed[20]);
console.log(discovered[20]);
console.log(parent[20]);

bfs(1);
console.log(parent);
listParent();
//find_path(1,20); //1,13,20
find_path(1,6); //1,2,3,4,5,6
find_path(1,12); //1,7,8,9,10,11,12
*/

// ref: Skiena, Programming Challenges, page 194
var processed = []; //which vertices have been processed
var discovered = []; //which vertices have been found
var parents = []; //discovery relation

// ref: Skiena, Programming Challenges, page 195
function initialize_search(){
    for (var i=1; i<=graph.nvertices; i++) {
        processed[i] = discovered[i] = false;
        parents[i] = -1;
    }
}

// ref: Skiena, Programming Challenges, page 196
function process_vertex(v){
    console.log("processed vertex ",v);
}

// ref: Skiena, Programming Challenges, page 196
function process_edge(x, y){
    console.log("processed edge (",x,",",y,")");
}

function valid_edge(e){
    if (e != "undefined") return (true);
    else return(false);
}

// ref: Skiena, Programming Challenges, page 195
function bfs(start){
    var v; //current vertex
    var i; //counter
    init_queue();
    enqueue(start);
    discovered[start] = true;
    while (empty() == false) {
        v = dequeue();
        process_vertex(v);
        processed[v] = true;
        for (var i=0; i<graph.degree[v]; i++){
            if (valid_edge(graph.edges[v][i]) == true) {
                if (discovered[graph.edges[v][i].v] == false) {
                    enqueue(graph.edges[v][i].v);
                    discovered[graph.edges[v][i].v] = true;
                    parents[graph.edges[v][i].v] = v;
                }
                if (processed[graph.edges[v][i].v] == false)
                    process_edge(v,graph.edges[v][i].v);
            }
        }
    }
}

//vertex:  1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 
//parent: -1 1 2 3 4 5 1 7 8  9 10 11  1 13 14 15 13 17 18 13 
function listParent(){
    var strVertex = "";
    var strParents = "";
    for (var i=1; i<=graph.nvertices; i++) {
        strVertex = strVertex + i + " ";
    }
    for (var i=1; i<=graph.nvertices; i++) {
        strParents = strParents + parents[i] + " ";
    }
    //console.log(strVertex);
    //console.log(strParents);
}

// ref: Skiena, Programming Challenges, page 197
var path = [];
function resetPath(){
    path = [];
}

function find_path(start, end){
    if ((start == end) || (end == -1)){
        path.push(start);
        //console.log(start);
    }
    else {
        find_path(start,parents[end]);
        path.push(end);
        //console.log(end);
    }
}

function getT(start,end){
    var gotEdge = false;
    //tree edge
    for(var i=0;i<graph.degree[start];i=i+1){
        var v = graph.edges[start][i].v;
        if(v == end){
            var T = graph.edges[start][i].T;
            gotEdge = true; //it is a tree edge
            //console.log('tree edge: ' + 'N' + start + ' -> ' + 'N' + end);
        }
    }
    //back edge
    if(!gotEdge){
        //flip 'start', 'end' nodes
        var tmp = start;
        start = end;
        end = tmp;
        for(var i=0;i<graph.degree[start];i=i+1){
            var v = graph.edges[start][i].v;
            if(v == end){
                var T = graph.edges[start][i].T;
                T = mcht.HTInverse(T);
                //console.log('back edge: ' + 'N' + end + ' -> ' + 'N' + start);
            }
        }
    }
  
    return T;
}

//Build homogenous transformation matrix.
//   - htma[] is an array of the 'A' matrix or 'A^-1' matrix.
//   - T0[] is the 'T' transform with reference to node at index 0 (zero).
//   - T0n is the 'T' matrix for last node with reference to node at index 0 (zero).
function TM(nodePath,T_BaseFrame){
    var htma = [];
    var T0 = [];
    for(var i=0;i<(nodePath.length - 1);i=i+1){
      var T = getT(nodePath[i],nodePath[i+1]);
      htma[i] = T;
      if(i == 0){
        var T0n = [
            [T[0][0], T[0][1], T[0][2], T[0][3]],
            [T[1][0], T[1][1], T[1][2], T[1][3]],
            [T[2][0], T[2][1], T[2][2], T[2][3]],
            [T[3][0], T[3][1], T[3][2], T[3][3]]
        ];
        T0[i] = hlao.matrix_multiplication(T_BaseFrame,T0n); //note: T_BaseFrame defined in Tu.js
      } else {
          T0n = hlao.matrix_multiplication(T0n,T);
          T0[i] = hlao.matrix_multiplication(T0[i-1],T);
          //console.log('T0[' + i + ']');
        }
    }
    
    return [htma, T0, T0n];
}

//Build homogenous transformation matrix.
//   - following based on htma[] and sol[]
//   - T0[] is the 'T' transform with reference to node at index 0 (zero).
//   - T0n is the 'T' matrix for last node with reference to node at index 0 (zero).
function TMhs(htma,sol,T_BaseFrame){
    var T0 = [];
    for(var i=0;i<n;i=i+1){
        var RPY = RPYangles(htma[i]); //get old roll angle (RPYangles() returns [[roll],[pitch],[yaw]])
        var phiB = RPY[0][0]; //roll
        var Tb = htma[i]; //transform beform the update
        var Tz = trotz(sol[15][i][0] + -1.0*phiB);
        var T = hlao.matrix_multiplication(Tz,Tb);
        //console.log(T);
        
        if(i == 0){
        var T0n = [
            [T[0][0], T[0][1], T[0][2], T[0][3]],
            [T[1][0], T[1][1], T[1][2], T[1][3]],
            [T[2][0], T[2][1], T[2][2], T[2][3]],
            [T[3][0], T[3][1], T[3][2], T[3][3]]
        ];
        T0[i] = hlao.matrix_multiplication(T_BaseFrame,T0n); //note: T_BaseFrame defined in Tu.js
        } else {
          T0n = hlao.matrix_multiplication(T0n,T);
          T0[i] = hlao.matrix_multiplication(T0[i-1],T);
        }
    }
    
    return [T0, T0n];
}

// Copy htma[] array.
function copyHTM(htma){
    var htmaTmp = [];
    
    var n = htma.length;
        
    //console.log('Copying htma[].');
    for(var i=0;i<n;i=i+1){
        htmaTmp[i] = mcht.DHDeepCopy(htma[i]);
    }
    //console.log('Copying htma[] complete.');
    
    return htmaTmp;
}

// Update htma[] array
function renew(htma,q){
    var n = htma.length;
    
    //console.log('Updating htma[].');
    for(var i=0;i<n;i=i+1){
        //update htma[]
        var RPY = RPYangles(htma[i]); //get old roll angle (RPYangles() returns [[roll],[pitch],[yaw]])
        var phiB = RPY[0][0]; //roll
        var Tb = htma[i]; //transform beform the update
        var Tz = trotz(q[i][0] + -1.0*phiB);
        var T = hlao.matrix_multiplication(Tz,Tb);
        htma[i] = mcht.DHDeepCopy(T);
    }
    //console.log('Updating htma[] complete.');
    
    return htma;
}

// Reverse the order of the homogenous transformation array.
function reverseHTM(htma){
    var n = htma.length;
    
    //console.log('Reverse htma[] order.');
    var htmaTmp = [];
    for(var i=(n-1);i>=0;i=i-1){
        //console.log('i: ' + i + ', (n-1)-i: ' + ((n-1)-i));
        htmaTmp[(n-1)-i] = mcht.DHDeepCopy(mcht.HTInverse(htma[i]));
    }
    //console.log(htmaTmp);
    
    return htmaTmp;
}

function buildTransforms(htmaTmp,T_BaseFrame){
    var n = htmaTmp.length;
    
    //console.log('Building T0[].');
    var T0 = [];
    for(var i=0;i<n;i=i+1){
        var T = htmaTmp[i];
        
        if(i == 0){
            var T0n = [
                [T[0][0], T[0][1], T[0][2], T[0][3]],
                [T[1][0], T[1][1], T[1][2], T[1][3]],
                [T[2][0], T[2][1], T[2][2], T[2][3]],
                [T[3][0], T[3][1], T[3][2], T[3][3]]
            ];
            //T0[i] = hlao.matrix_multiplication(T_BaseFrame,T0n); //note: T_BaseFrame defined in Tu.js
            T0[i] = mcht.DHDeepCopy(T0n);
        } else {
            T0n = hlao.matrix_multiplication(T0n,T);
            T0[i] = hlao.matrix_multiplication(T0[i-1],T);
        }
    }
    //console.log('Finished building T0[].');
    
    return [T0, T0n];
}

// With T_BaseFrame[]
function buildTransformsBaseFrame(htmaTmp,T_BaseFrame){
    var n = htmaTmp.length;
    
    //console.log('Building T0[].');
    var T0 = [];
    for(var i=0;i<n;i=i+1){
        var T = htmaTmp[i];
        
        if(i == 0){
            var T0n = [
                [T[0][0], T[0][1], T[0][2], T[0][3]],
                [T[1][0], T[1][1], T[1][2], T[1][3]],
                [T[2][0], T[2][1], T[2][2], T[2][3]],
                [T[3][0], T[3][1], T[3][2], T[3][3]]
            ];
            T0[i] = hlao.matrix_multiplication(T_BaseFrame,T0n); //note: T_BaseFrame defined in Tu.js
            //T0[i] = mcht.DHDeepCopy(T0n);
        } else {
            T0n = hlao.matrix_multiplication(T0n,T);
            T0[i] = hlao.matrix_multiplication(T0[i-1],T);
        }
    }
    //console.log('Finished building T0[].');
    
    return [T0, T0n];
}

// Update htma, T0, T0n
function updateHTM(htma,q,T_BaseFrame){
    var n = htma.length;
    //console.log('n: ' + n);
    
    //   - update htma[]
    var htma = renew(htma,q);
    
    //   - reverse htma[] array
    var htma = reverseHTM(htma);
    
    //   - copy over to htma[]
    var builtTransforms = buildTransforms(htma);
    var T0 = builtTransforms[0];
    var T0n = builtTransforms[1];
    
    return [htma, T0, T0n];
}

var T0nOLD = [
   [1.0, 0.0, 0.0, 0.0],
   [0.0, 1.0, 0.0, 0.0],
   [0.0, 0.0, 1.0, 0.0],
   [0.0, 0.0, 0.0, 1.0]
];

var showFrame = false;

export {
    MAXV,
    MAXDEGREE,
    graph,
    initialize_graph,
    insert_edge,
    print_graph,
    generate_graph,
    QUEUESIZE,
    queue,
    init_queue,
    enqueue,
    dequeue,
    empty,
    processed,
    discovered,
    parents,
    initialize_search,
    process_vertex,
    process_edge,
    valid_edge,
    bfs,
    listParent,
    path,
    resetPath,
    find_path,
    getT,
    TM,
    TMhs,
    copyHTM,
    renew,
    reverseHTM,
    buildTransforms,
    buildTransformsBaseFrame,
    updateHTM,
    T0nOLD,
    showFrame
};