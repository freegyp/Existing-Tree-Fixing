
//
// Disclamer:
// ----------
//
// This code will work only if you selected window, graphics and audio.
//
// Note that the "Run Script" build phase will copy the required frameworks
// or dylibs to your application bundle so you can execute it on any OS X
// computer.
//
// Your resource files (images, sounds, fonts, ...) are also copied to your
// application bundle. To get the path to these resource, use the helper
// method resourcePath() from ResourcePath.hpp
//

#include <SFML/Graphics.hpp>
#include <vector>
#include <set>
#include <map>
#include <utility>
#include <iostream>
#include <cmath>
#include <fstream>
// Here is a small helper for you ! Have a look.
#include "ResourcePath.hpp"
using namespace std;

struct MSTNode{
    int ID;
    sf::Vector2f position;
    sf::Vector2f tangentVector=sf::Vector2f(0,0);
    sf::Vector2f normal;
    sf::Vector2f centroid;
    float traveled_distance;
    float local_radius;
    float cutLength=0.0f;
    bool inVagueJunction=false;
    vector<MSTNode*> neighbours;
    vector<MSTNode*> junctionVerges;
    int num_of_neighbours;
    bool overlap=false;
    int overlapID=-1;
    MSTNode(sf::Vector2f pos,int id){
        position=pos;
        ID=id;
        num_of_neighbours=0;
    }
    void calcTangentForLeaf(){
        sf::Vector2f center=position-neighbours[0]->position;
        center/=sqrt(center.x*center.x+center.y*center.y);
        center+=center-neighbours[0]->tangentVector;
        center/=sqrt(center.x*center.x+center.y*center.y);
        tangentVector=center;
        normal=sf::Vector2f(-tangentVector.y,tangentVector.x);
    }
    void calcTangent(){
        if(num_of_neighbours==2){
            float coefficient=1;
            sf::Vector2f v1=position-neighbours[0]->position;
            if(neighbours[0]->num_of_neighbours==1){
                coefficient*=-1;
            }
            float l1=sqrt(v1.x*v1.x+v1.y*v1.y);
            if(l1!=0)v1/=l1;
            else{
                for(vector<MSTNode*>::iterator it=neighbours[0]->neighbours.begin();it!=neighbours[0]->neighbours.end();it++){
                    if((*it)->ID!=ID){
                        v1=position-(*it)->position;
                        l1=sqrt(v1.x*v1.x+v1.y*v1.y);
                        if(l1!=0)v1/=l1;
                        else cout<<"Consecutive zero lines!"<<endl;
                    }
                }
            }
            sf::Vector2f v2=neighbours[1]->position-position;
            float l2=sqrt(v2.x*v2.x+v2.y*v2.y);
            if(l2!=0)v2/=l2;
            else{
                for(vector<MSTNode*>::iterator it=neighbours[1]->neighbours.begin();it!=neighbours[1]->neighbours.end();it++){
                    if((*it)->ID!=ID){
                        v2=position-(*it)->position;
                        l2=sqrt(v2.x*v2.x+v2.y*v2.y);
                        if(l2!=0)v2/=l2;
                        else cout<<"Consecutive zero lines!"<<endl;
                    }
                }
            }
            sf::Vector2f bigV=v1*l2/(l1+l2)+v2*l1/(l1+l2);
            tangentVector=bigV*coefficient;
            normal=sf::Vector2f(-tangentVector.y,tangentVector.x);
        }
        else if (num_of_neighbours==3){
            sf::Vector2f v1=position-neighbours[0]->position;
            float l1=sqrt(v1.x*v1.x+v1.y*v1.y);
            if(l1!=0)v1/=l1;
            else cout<<"Zero line found!!"<<endl;
            sf::Vector2f v2=neighbours[1]->position-position;
            float l2=sqrt(v2.x*v2.x+v2.y*v2.y);
            if(l2!=0)v2/=l2;
            else cout<<"Zero line found!!"<<endl;
            sf::Vector2f v3=neighbours[2]->position-position;
            float l3=sqrt(v3.x*v3.x+v3.y*v3.y);
            if(l3!=0)v3/=l3;
            else cout<<"Zero line found!!"<<endl;
            float dot12=abs(v1.x*v2.x+v1.y*v2.y),dot23=abs(v2.x*v3.x+v2.y*v3.y),dot13=abs(v1.x*v3.x+v1.y*v3.y);
            sf::Vector2f left,right;
            sf::Vector2f bigV;
            if(dot12>=dot23&&dot12>=dot13){
                bigV=v1*l2/(l1+l2)+v2*l1/(l1+l2);
            }
            else if(dot23>=dot12&&dot23>=dot13){
                bigV=v2*l3/(l2+l3)+v3*l2/(l2+l3);
            }
            else if (dot13>=dot12&&dot13>=dot23){
                bigV=v1*l3/(l1+l3)+v3*l1/(l1+l3);
            }
            tangentVector=bigV;
            normal=sf::Vector2f(-tangentVector.y,tangentVector.x);
        }
    }
    bool isInTree(int id,set<int>* gone){
        gone->insert(ID);
        for(int i=0;i<num_of_neighbours;i++){
            if(gone->count(neighbours[i]->ID)==0){
                if(neighbours[i]->ID==id)return true;
                if(neighbours[i]->isInTree(id, gone))return true;
            }
        }
        return false;
    }
    bool addNeighbour(MSTNode* neighbour){
        if(num_of_neighbours!=0)
            for(int i=0;i<num_of_neighbours;i++){
                if(neighbours[i]==neighbour){
                    return false;
                }
            }
        neighbours.push_back(neighbour);
        num_of_neighbours++;
        neighbour->addNeighbour(this);
        return true;
    }
    bool cutThisLeaf(){
        if(num_of_neighbours==1){
            if(cutLength<local_radius){
                for(int i=0;i<neighbours[0]->num_of_neighbours;i++){
                    if(ID==neighbours[0]->neighbours[i]->ID){
                        neighbours[0]->neighbours.erase(neighbours[0]->neighbours.begin()+i);
                        neighbours[0]->num_of_neighbours-=1;
                        sf::Vector2f diff=position-neighbours[0]->position;
                        float d=sqrt(diff.x*diff.x+diff.y*diff.y);
                        neighbours[0]->cutLength+=d+cutLength;
                        break;
                    }
                }
                neighbours.clear();
                num_of_neighbours--;
                return true;
            }
            else return false;
        }
        else{
            return false;
        }
    }
    
    void fillNeighbouringGap(vector<MSTNode*>* nodes, set<int>* gone){
        gone->insert(ID);
        for(int i=0;i<num_of_neighbours;i++){
            if(gone->find(neighbours[i]->ID)==gone->end()){
                sf::Vector2f diff=neighbours[i]->position-position;
                float dist=sqrt(diff.x*diff.x+diff.y*diff.y);
                if(dist>2.0){
                    int maxID=nodes->size();
                    sf::Vector2f newPos=position+(diff/dist);
                    MSTNode* tofill=new MSTNode(newPos,maxID);
                    MSTNode* prev=neighbours[i];
                    neighbours[i]=tofill;
                    for(int j=0;j<prev->num_of_neighbours;j++){
                        if(prev->neighbours[j]==this){
                            prev->neighbours[j]=tofill;
                        }
                    }
                    tofill->addNeighbour(this);
                    tofill->addNeighbour(prev);
                    tofill->local_radius=local_radius+(prev->local_radius-local_radius)/dist;
                    nodes->push_back(tofill);
                    neighbours[i]->fillNeighbouringGap(nodes, gone);
                }
                else{
                    neighbours[i]->fillNeighbouringGap(nodes, gone);
                }
            }
        }
    }
    
    void junctionPointing(){
        if(num_of_neighbours<=2)return;
        float r=local_radius;
        bool done=false;
        vector<MSTNode*> intersections=neighbours;
        vector<MSTNode*> prevs(num_of_neighbours,this);
        while(!done){
            done=true;
            for(int i=0;i<num_of_neighbours;i++){
                sf::Vector2f diff=intersections[i]->position-position;
                while(sqrt(diff.x*diff.x+diff.y*diff.y)<r){
                    prevs[i]->inVagueJunction=true;
                    MSTNode* temp;
                    for(int j=0;j<intersections[i]->num_of_neighbours;j++){
                        if(intersections[i]->neighbours[j]!=prevs[i]){
                            temp=intersections[i]->neighbours[j];
                        }
                    }
                    prevs[i]=intersections[i];
                    intersections[i]=temp;
                    diff=intersections[i]->position-position;
                }
                prevs[i]->inVagueJunction=true;
            }
            sf::Vector2f diff1=intersections[0]->position-intersections[1]->position;
            sf::Vector2f diff2=intersections[0]->position-intersections[2]->position;
            sf::Vector2f diff3=intersections[1]->position-intersections[2]->position;
            if(sqrt(diff1.x*diff1.x+diff1.y*diff1.y)<intersections[0]->local_radius+intersections[1]->local_radius)done=false;
            if(sqrt(diff2.x*diff2.x+diff2.y*diff2.y)<intersections[0]->local_radius+intersections[2]->local_radius)done=false;
            if(sqrt(diff3.x*diff3.x+diff3.y*diff3.y)<intersections[1]->local_radius+intersections[2]->local_radius)done=false;
            if(!done)r+=0.05;
        }
        junctionVerges=intersections;
    }
    
    void seperateFromThisHere(vector<MSTNode*>* nodes,MSTNode* prev=NULL){
        for(int i=0;i<num_of_neighbours;i++){
            if(neighbours[i]!=prev){
                if(neighbours[i]->num_of_neighbours>2){
                    sf::Vector2f inTangent=neighbours[i]->position-position;
                    inTangent/=sqrt(inTangent.x*inTangent.x+inTangent.y*inTangent.y);
                    for(vector<MSTNode*>::iterator it=neighbours[i]->neighbours.begin();it!=neighbours[i]->neighbours.end();it++){
                        if((*it)==this){
                            neighbours[i]->neighbours.erase(it);
                            neighbours[i]->num_of_neighbours--;
                            break;
                        }
                    }
                    int maxIndex;
                    float maxvalue=-5;
                    for(int j=0;j<neighbours[i]->num_of_neighbours;j++){
                        sf::Vector2f outTangent=neighbours[i]->neighbours[j]->position-neighbours[i]->position;
                        outTangent/=sqrt(outTangent.x*outTangent.x+outTangent.y*outTangent.y);
                        if(outTangent.x*inTangent.x+outTangent.y*inTangent.y>maxvalue){
                            maxvalue=outTangent.x*inTangent.x+outTangent.y*inTangent.y;
                            maxIndex=j;
                        }
                    }
                    MSTNode* toPush=new MSTNode(neighbours[i]->position,nodes->size());
                    toPush->addNeighbour(neighbours[i]->neighbours[maxIndex]);
                    for(int j=0;j<neighbours[i]->num_of_neighbours;j++){
                        if(j==maxIndex){
                            for(int k=0;k<neighbours[i]->neighbours[j]->num_of_neighbours;k++){
                                if(neighbours[i]->neighbours[j]->neighbours[k]==neighbours[i]){
                                    neighbours[i]->neighbours[j]->neighbours.erase(neighbours[i]->neighbours[j]->neighbours.begin()+k);
                                    neighbours[i]->neighbours[j]->num_of_neighbours--;
                                    break;
                                }
                            }
                            neighbours[i]->neighbours.erase(neighbours[i]->neighbours.begin()+maxIndex);
                            neighbours[i]->num_of_neighbours--;
                            break;
                        }
                    }
                    neighbours[i]=toPush;
                    toPush->addNeighbour(this);
                    nodes->push_back(toPush);
                }
                else{
                    neighbours[i]->seperateFromThisHere(nodes,this);
                }
            }
        }
    }
    
    void drawFromThisNode(sf::RenderWindow* window,set<int>* gone,set<int>* blacklist,float xTranslate=0,float yTranslate=0,sf::Color color=sf::Color::Black,sf::PrimitiveType type=sf::PrimitiveType::Lines){
        gone->insert(ID);
        for(int i=0;i<num_of_neighbours;i++){
            if(gone->find(neighbours[i]->ID)==gone->end()){
                if(blacklist->find(neighbours[i]->ID)!=blacklist->end())continue;
                sf::VertexArray line;
                line.append(sf::Vertex(position+sf::Vector2f(xTranslate,yTranslate),color));
                line.append(sf::Vertex(neighbours[i]->position+sf::Vector2f(xTranslate,yTranslate),color));
                line.setPrimitiveType(type);
                window->draw(line);
                neighbours[i]->drawFromThisNode(window,gone,blacklist,xTranslate,yTranslate,color,type);
            }
        }
    }
    
    void outputFromThisNode(ofstream &fout,set<int>* gone){
        gone->insert(ID);
        fout<<position.x<<" "<<position.y<<endl;
        for(int i=0;i<num_of_neighbours;i++){
            if(gone->find(neighbours[i]->ID)==gone->end()){
                neighbours[i]->outputFromThisNode(fout, gone);
            }
        }
    }
};

void moveGraph(vector<MSTNode*>* nodes,float r=6.0f){
    for (vector<MSTNode*>::iterator it=nodes->begin(); it!=nodes->end(); it++) {
        if ((*it)->num_of_neighbours==0) {
            continue;
        }
        (*it)->calcTangent();
        vector<float> weights;
        vector<sf::Vector2f> positions;
        int num_of_points=0;
        for (vector<MSTNode*>::iterator it2=nodes->begin(); it2!=nodes->end(); it2++) {
            if ((*it2)->num_of_neighbours==0) continue;
            sf::Vector2f diff=(*it2)->position-(*it)->position;
            if (diff.x*diff.x+diff.y*diff.y<=r*r) {
                num_of_points++;
                float weight=exp(-sqrt(diff.x*diff.x+diff.y*diff.y)/(2*(*it)->local_radius*(*it)->local_radius));
                weights.push_back(weight);
                positions.push_back((*it2)->position);
            }
        }
        sf::Vector2f centroid=sf::Vector2f(0,0);
        float sum_weight=0;
        for (int i=0; i<weights.size(); i++) {
            centroid+=weights[i]*positions[i];
            sum_weight+=weights[i];
        }
        centroid/=sum_weight;
        (*it)->centroid=centroid;
    }
    for (vector<MSTNode*>::iterator it=nodes->begin(); it!=nodes->end(); it++) {
        if ((*it)->num_of_neighbours==1) {
            (*it)->calcTangentForLeaf();
        }
    }
    for (vector<MSTNode*>::iterator it=nodes->begin(); it!=nodes->end(); it++) {
        if ((*it)->num_of_neighbours==0) {
            continue;
        }
        sf::Vector2f cpdiff=(*it)->centroid-(*it)->position;
        sf::Vector2f plus=(cpdiff.x*(*it)->normal.x+cpdiff.y*(*it)->normal.y)*(*it)->normal;
        //plus*=0.5f;
        (*it)->position+=plus;
    }
}

void reJunction(vector<MSTNode*>* nodes){
    vector<MSTNode*> junctions;
    for (vector<MSTNode*>::iterator it=nodes->begin(); it!=nodes->end(); it++) {
        if ((*it)->num_of_neighbours>2) {
            junctions.push_back((*it));
        }
    }
    for (vector<MSTNode*>::iterator it=junctions.begin(); it!=junctions.end(); it++) {
        vector<MSTNode*> intersections=(*it)->junctionVerges;
        vector<sf::Vector2f> tangents;
        for (vector<MSTNode*>::iterator it2=intersections.begin(); it2!=intersections.end(); it2++) {
            for (int i=0; i<(*it2)->num_of_neighbours; i++) {
                if ((*it2)->neighbours[i]->inVagueJunction) {
                    sf::Vector2f tangentLine=(*it2)->neighbours[i]->position-(*it2)->position;
                    tangentLine/=sqrt(tangentLine.x*tangentLine.x+tangentLine.y*tangentLine.y);
                    tangents.push_back(tangentLine);
                }
            }
        }
        vector<MSTNode*> curPoints=(*it)->neighbours;
        for (int i=0; i<(*it)->num_of_neighbours; i++) {
            for (vector<MSTNode*>::iterator it2=(*it)->neighbours[i]->neighbours.begin(); it2!=(*it)->neighbours[i]->neighbours.end(); it2++) {
                if ((*it2)==(*it)) {
                    (*it)->neighbours[i]->neighbours.erase(it2);
                    (*it)->neighbours[i]->num_of_neighbours--;
                    break;
                }
            }
        }
        (*it)->neighbours.clear();
        (*it)->num_of_neighbours=0;
        for (vector<MSTNode*>::iterator it2=curPoints.begin(); it2!=curPoints.end(); it2++) {
            bool cutDone=false;
            while (!cutDone) {
                MSTNode* prev=(*it2);
                (*it2)=(*it2)->neighbours[0];
                for (vector<MSTNode*>::iterator it3=(*it2)->neighbours.begin(); it3!=(*it2)->neighbours.end(); it3++) {
                    if ((*it3)==prev) {
                        (*it2)->neighbours.erase(it3);
                        (*it2)->num_of_neighbours--;
                        break;
                    }
                }
                prev->num_of_neighbours=0;
                prev->neighbours.clear();
                if (!((*it2)->inVagueJunction)) {
                    cutDone=true;
                }
            }
        }
        for (int i=0; i<intersections.size(); i++) {
            if (intersections[i]->num_of_neighbours!=1) {
                cout<<"Problem!"<<endl;
            }
        }
        for (int i=0; i<intersections.size(); i++) {
            for (int j=i+1; j<intersections.size(); j++) {
                float product=-tangents[i].x*tangents[j].x-tangents[i].y*tangents[j].y;
                if (product<sqrt(2)/2) {
                    continue;
                }
                MSTNode* current=intersections[i];
                for (float s=0.02; s<1.0; s+=0.02) {
                 sf::Vector2f h1=(float)(2.0*pow(s, 3.0)-3.0*pow(s, 2.0)+1.0)*intersections[i]->position;
                 sf::Vector2f h2=(float)(-2*pow(s, 3.0)+3*pow(s, 2.0))*intersections[j]->position;
                 sf::Vector2f h3=(float)(pow(s, 3.0)-2*pow(s, 2.0)+s)*(1.0f*tangents[i]);
                 sf::Vector2f h4=(float)(pow(s, 3.0)-pow(s, 2.0))*(-1.0f*tangents[j]);
                 sf::Vector2f newPos=h1+h2+h3+h4;
                 MSTNode* toPush=new MSTNode(newPos,nodes->size());
                 if(current->addNeighbour(toPush));
                 else cout<<"Neighbour adding failed."<<endl;
                 current=toPush;
                 nodes->push_back(current);
                 }
                if(!current->addNeighbour(intersections[j]))cout<<"Neighbour adding failed."<<endl;
            }
        }
        for (int i=0; i<intersections.size(); i++) {
            if (intersections[i]->num_of_neighbours==3) {
                bool found_in_nodes=false;
                for (int j=0; j<nodes->size(); j++) {
                    if ((*nodes)[j]==intersections[i]) {
                        found_in_nodes=true;
                        break;
                    }
                }
                if (found_in_nodes) {
                    cout<<"Found intersection in nodes!"<<endl;
                }
            }
        }
    }
}

int main(int, char const**)
{
    // Create the main window
    sf::RenderWindow window(sf::VideoMode(1280, 800), "SFML window");

    // Set the Icon
    sf::Image icon;
    if (!icon.loadFromFile(resourcePath() + "icon.png")) {
        return EXIT_FAILURE;
    }
    window.setIcon(icon.getSize().x, icon.getSize().y, icon.getPixelsPtr());

    vector<MSTNode*> nodes;
    ifstream fin1(resourcePath()+"points.in",ifstream::in);
    int check=0;
    while (!fin1.eof()) {
        int id;
        float x,y,distance;
        fin1>>id>>x>>y>>distance;
        if (id!=check) {
            cout<<"Misplacement found on file at "<<check<<endl;
        }
        check++;
        MSTNode* nodei=new MSTNode(sf::Vector2f(x,y),id);
        nodei->traveled_distance=distance;
        nodes.push_back(nodei);
    }
    fin1.close();
    cout<<"On read the file, there are "<<nodes.size()<<" points."<<endl;
    
    ifstream fin2(resourcePath()+"topology.in",ifstream::in);
    while (!fin2.eof()) {
        int leftID,rightID;
        fin2>>leftID>>rightID;
        nodes[leftID]->addNeighbour(nodes[rightID]);
    }
    fin2.close();
    for (vector<MSTNode*>::iterator it=nodes.begin(); it!=nodes.end(); it++) {
        float maxDist=(*it)->traveled_distance;
        for (vector<MSTNode*>::iterator it2=nodes.begin(); it2!=nodes.end(); it2++) {
            sf::Vector2f diff=(*it)->position-(*it2)->position;
            float d=sqrt(diff.x*diff.x+diff.y*diff.y);
            if (d<=2&&(*it2)->traveled_distance>maxDist) {
                maxDist=(*it2)->traveled_distance;
            }
        }
        (*it)->local_radius=maxDist;
    }
    set<int> gone_nodes;
    set<int> blacklist;
    bool pruneDone=false;
    while (!pruneDone) {
        pruneDone=true;
        for (vector<MSTNode*>::iterator it=nodes.begin(); it!=nodes.end(); it++) {
            if ((*it)->num_of_neighbours==1) {
                if((*it)->cutThisLeaf())pruneDone=false;
            }
        }
    }
    map<int,MSTNode*> leafSet;
    for (vector<MSTNode*>::iterator it=nodes.begin(); it!=nodes.end(); it++) {
        if ((*it)->num_of_neighbours==1) {
            leafSet.insert(pair<int, MSTNode*>((*it)->ID,(*it)));
        }
    }
    for (map<int,MSTNode*>::iterator it=leafSet.begin(); it!=leafSet.end(); it++) {
        map<int,MSTNode*> locals;
        bool twoLeaves=false;
        for (vector<MSTNode*>::iterator it2=nodes.begin(); it2!=nodes.end(); it2++) {
            sf::Vector2f diff=(*it).second->position-(*it2)->position;
            float d=sqrt(diff.x*diff.x+diff.y*diff.y);
            if (d<=(*it).second->local_radius*4) {
                MSTNode* nodei=new MSTNode((*it2)->position,(*it2)->ID);
                nodei->overlap=(*it2)->overlap;
                nodei->overlapID=(*it2)->overlapID;
                locals.insert(pair<int, MSTNode*>(nodei->ID,nodei));
                if (nodei->ID!=(*it).second->ID&&leafSet.find(nodei->ID)!=leafSet.end()) {
                    twoLeaves=true;
                }
            }
        }
        if (!twoLeaves) {
            continue;
        }
        multimap<float,pair<int,int>> possible_edges;
        for (map<int,MSTNode*>::iterator it2=locals.begin(); it2!=locals.end(); it2++) {
            map<int,MSTNode*>::iterator it3=it2;
            it3++;
            while (it3!=locals.end()) {
                int leftID=(*it2).second->ID,rightID=(*it3).second->ID;
                sf::Vector2f diff=locals[leftID]->position-locals[rightID]->position;
                float distance=sqrt(diff.x*diff.x+diff.y*diff.y);
                bool inGlobal=false;
                for (vector<MSTNode*>::iterator it4=nodes[(*it2).second->ID]->neighbours.begin(); it4!=nodes[(*it2).second->ID]->neighbours.end(); it4++) {
                    if ((*it4)->ID==(*it3).second->ID) {
                        inGlobal=true;
                    }
                }
                if (inGlobal) distance=0;
                possible_edges.insert(pair<float,pair<int,int>>(distance,pair<int, int>(leftID,rightID)));
                it3++;
            }
        }
        for (multimap<float,pair<int,int>>::iterator it2=possible_edges.begin(); it2!=possible_edges.end(); it2++)
        {
            gone_nodes.clear();
            if (!locals[(*it2).second.first]->isInTree((*it2).second.second, &gone_nodes)) {
                locals[(*it2).second.first]->addNeighbour(locals[(*it2).second.second]);
                nodes[(*it2).second.first]->addNeighbour(nodes[(*it2).second.second]);
            }
        }
        for (map<int,MSTNode*>::iterator it2=locals.begin(); it2!=locals.end(); it2++) {
            nodes[(*it2).first]->cutLength=0;
        }
        if (twoLeaves) {
            break;
        }
    }
    pruneDone=false;
    int leaves=0,junctions=0;
    while (!pruneDone) {
        pruneDone=true;
        leaves=0;
        junctions=0;
        for (vector<MSTNode*>::iterator it=nodes.begin(); it!=nodes.end(); it++) {
            if ((*it)->num_of_neighbours==1) {
                if((*it)->cutThisLeaf())pruneDone=false;
                leaves++;
            }
            else if ((*it)->num_of_neighbours>2){
                for (vector<MSTNode*>::iterator it2=(*it)->neighbours.begin(); it2!=(*it)->neighbours.end(); it2++) {
                    sf::Vector2f vec=(*it)->position-(*it2)->position;
                    vec/=sqrt(vec.x*vec.x+vec.y*vec.y);
                }
                junctions++;
            }
        }
    }
    
    cout<<"The number of leaves is "<<leaves<<"."<<endl<<"And the number of junctions is "<<junctions<<"."<<endl;
    float xTranslate=0,yTranslate=0;
    bool iterationStop=false;
    int iterationNO=0;
    sf::Color color_of_lines=sf::Color::Black;
    vector<MSTNode*> original;
    float rr=2.0f;
    sf::CircleShape circle;
    circle.setRadius(8);
    circle.setOutlineThickness(1);
    circle.setOutlineColor(sf::Color::Black);
    sf::PrimitiveType type=sf::PrimitiveType::Lines;
    // Start the game loop
    while (window.isOpen())
    {
        // Process events
        sf::Event event;
        while (window.pollEvent(event))
        {
            // Close window : exit
            if (event.type == sf::Event::Closed) {
                window.close();
            }

            // Espace pressed : exit
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Escape) {
                window.close();
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down)) {
                yTranslate-=100;
            }
            else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up)){
                yTranslate+=100;
            }
            else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left)){
                xTranslate-=100;
            }
            else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right)){
                xTranslate+=100;
            }
            else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Return)){
                gone_nodes.clear();
                nodes[0]->fillNeighbouringGap(&nodes, &gone_nodes);
                moveGraph(&nodes,rr);
                iterationNO++;
            }
        }

        
        // Clear screen
        window.clear(sf::Color::White);
        if (iterationNO<=30) {
            gone_nodes.clear();
            nodes[0]->fillNeighbouringGap(&nodes, &gone_nodes);
            moveGraph(&nodes,rr);
            iterationNO++;
        }
        else{
            color_of_lines=sf::Color::Red;
            //window.close();
        }

        gone_nodes.clear();
        //circle.setPosition(sf::Mouse::getPosition().x, sf::Mouse::getPosition().y);
        //window.draw(circle);
        nodes[1]->drawFromThisNode(&window, &gone_nodes, &blacklist,xTranslate,yTranslate,color_of_lines,type);

        // Update the window
        window.display();
    }
    cout<<iterationNO<<" times iterations."<<endl;
    cout<<"And there are "<<nodes.size()<<" points right now."<<endl;
    for (vector<MSTNode*>::iterator it=nodes.begin(); it!=nodes.end(); it++) {
        if ((*it)->num_of_neighbours>2) {
            (*it)->junctionPointing();
        }
    }
    reJunction(&nodes);
    leaves=0;
    junctions=0;
    int sep=0,rightDisconnect=0;
    for (vector<MSTNode*>::iterator it=nodes.begin(); it!=nodes.end(); it++) {
        if ((*it)->num_of_neighbours>2) {
            junctions++;
        }
        else if ((*it)->num_of_neighbours==1){
            leaves++;
        }
        else if ((*it)->num_of_neighbours==0){
            sep++;
        }
        if ((*it)->inVagueJunction) {
            rightDisconnect++;
        }
    }
    cout<<"Found "<<leaves<<" leaves, "<<junctions<<" junctions, "<<sep<<" seperated points and "<<rightDisconnect<<" disconnected points."<<endl;
    sf::RenderWindow window2(sf::VideoMode(1280, 800), "Second window");
    window2.setIcon(icon.getSize().x, icon.getSize().y, icon.getPixelsPtr());
    sf::VertexArray array;
    for (int i=0; i<nodes.size(); i++) {
        if (nodes[i]->ID!=i) {
            cout<<"ID number error at "<<i<<"th point, the ID in the structure is "<<nodes[i]->ID<<"."<<endl;
        }
        if (nodes[i]->num_of_neighbours==0) {
            continue;
        }
        sf::Vertex v(nodes[i]->position,sf::Color::Black);
        array.append(v);
    }
    array.setPrimitiveType(sf::PrimitiveType::Points);
    ofstream fout(resourcePath()+"LineSegments.svg");
    vector<pair<sf::Vector2f, sf::Vector2f>> linesegs;
    for (int i=0; i<nodes.size(); i++) {
        for (int j=0; j<nodes[i]->num_of_neighbours; j++) {
            bool alreadyPushed=false;
            /*for (int k=0; k<linesegs.size(); k++) {
                if (linesegs[k].first==nodes[i]->position&&linesegs[k].second==nodes[i]->neighbours[j]->position) {
                    alreadyPushed=true;
                    break;
                }
                if (linesegs[k].second==nodes[i]->position&&linesegs[k].first==nodes[i]->neighbours[j]->position) {
                    alreadyPushed=true;
                    break;
                }
            }*/
            if (!alreadyPushed) {
                linesegs.push_back(pair<sf::Vector2f, sf::Vector2f>(nodes[i]->position,nodes[i]->neighbours[j]->position));
            }
        }
    }
    fout<<"<svg version=\"1.1\" baseProfile=\"full\" width=\"696\" height=\"1066\" xmlns=\"http://www.w3.org/2000/svg\">"<<endl;
    fout<<" <path stroke=\"red\" stroke-width=\"1\" d=\"";
    for (int i=0; i<linesegs.size(); i++) {
        fout<<"M"<<linesegs[i].first.x<<","<<linesegs[i].first.y<<" L "<<linesegs[i].second.x<<","<<linesegs[i].second.y<<" ";
    }
    cout<<resourcePath()<<endl;
    fout<<"\" />"<<endl;
    fout<<"</svg>"<<endl;
    fout.close();
    vector<MSTNode*> leavesvec;
    for (vector<MSTNode*>::iterator it=nodes.begin(); it!=nodes.end(); it++) {
        if ((*it)->num_of_neighbours==1) {
            leavesvec.push_back((*it));
        }
    }
    for (vector<MSTNode*>::iterator it=leavesvec.begin(); it!=leavesvec.end(); it++) {
        (*it)->seperateFromThisHere(&nodes);
    }
    vector<MSTNode*> toDraw;
    while (window2.isOpen()) {
        sf::Event event;
        while (window2.pollEvent(event))
        {
            // Close window : exit
            if (event.type == sf::Event::Closed) {
                window2.close();
            }
            
            // Espace pressed : exit
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Escape) {
                window2.close();
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down)) {
                for (int i=0; i<array.getVertexCount(); i++) {
                    array[i].position+=sf::Vector2f(0,-100);
                }
                yTranslate-=100;
            }
            else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up)){
                for (int i=0; i<array.getVertexCount(); i++) {
                    array[i].position+=sf::Vector2f(0,100);
                }
                yTranslate+=100;
            }
            else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left)){
                for (int i=0; i<array.getVertexCount(); i++) {
                    array[i].position+=sf::Vector2f(-100,0);
                }
                xTranslate-=100;
            }
            else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right)){
                for (int i=0; i<array.getVertexCount(); i++) {
                    array[i].position+=sf::Vector2f(100,0);
                }
                xTranslate+=100;
            }
            if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Left)) {
                for (int i=0; i<nodes.size(); i++) {
                    sf::Vector2f diff=nodes[i]->position+sf::Vector2f(xTranslate,yTranslate)-(sf::Vector2f)(sf::Mouse::getPosition(window2));
                    if (sqrt(diff.x*diff.x+diff.y*diff.y)<8&&nodes[i]->num_of_neighbours==1) {
                        toDraw.push_back(nodes[i]);
                    }
                }
            }
        }
        window2.clear(sf::Color::White);
        gone_nodes.clear();
        blacklist.clear();
        circle.setPosition(sf::Mouse::getPosition(window2).x-8, sf::Mouse::getPosition(window2).y-8);
        window2.draw(circle);
        for (int i=0; i<linesegs.size(); i++) {
            sf::Vertex v1(1.0f*linesegs[i].first+sf::Vector2f(xTranslate,yTranslate),sf::Color::Black);
            sf::Vertex v2(1.0f*linesegs[i].second+sf::Vector2f(xTranslate,yTranslate),sf::Color::Black);
            sf::VertexArray theLine;
            theLine.append(v1);
            theLine.append(v2);
            theLine.setPrimitiveType(sf::PrimitiveType::Lines);
            window2.draw(theLine);
        }
        for (vector<MSTNode*>::iterator it=toDraw.begin(); it!=toDraw.end(); it++) {
            (*it)->drawFromThisNode(&window2, &gone_nodes, &blacklist,xTranslate,yTranslate,color_of_lines,type);
        }
        
        //window2.draw(array);
        window2.display();
    }
    
    junctions=0;
    leaves=0;
    sep=0;
    rightDisconnect=0;
    
    for (vector<MSTNode*>::iterator it=nodes.begin(); it!=nodes.end(); it++) {
        if ((*it)->num_of_neighbours>2) {
            junctions++;
        }
        else if ((*it)->num_of_neighbours==1){
            leaves++;
        }
        else if ((*it)->num_of_neighbours==0){
            sep++;
        }
        if ((*it)->inVagueJunction) {
            rightDisconnect++;
        }
    }
    cout<<"Found "<<leaves<<" leaves, "<<junctions<<" junctions, "<<sep<<" seperated points and "<<rightDisconnect<<" disconnected points."<<endl;
    
    ofstream fout2(resourcePath()+"lineData.out",ofstream::out);
    gone_nodes.clear();
    for (vector<MSTNode*>::iterator it=toDraw.begin(); it!=toDraw.end(); it++) {
        if (gone_nodes.find((*it)->ID)==gone_nodes.end()) {
            //fout2<<"PathBegin"<<endl;
            (*it)->outputFromThisNode(fout2, &gone_nodes);
            fout2<<"PathEnd"<<endl;
        }
    }
    fout2.close();
    
    return EXIT_SUCCESS;
}
