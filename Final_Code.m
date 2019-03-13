%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% NETWORK TOPOLOGY SIMULATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%% The code is able to cover all the nodes in a network at %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% different power level. The entire network %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% is able to communicate with each other at higher %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% transmit power. %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%% The network is optimized at different power level for different %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%% clusters. The overall power consumption for each cluster has %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%% reduced, thus exapanding the battery life of anetwork.%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
clear;
clc;

%%%%%%%%%%%%%%%%%%-------Global variables---------%%%%%%%%%%%%%%%%%%
global n; %%%%%%%% To come out of recursion when n =300 %%%%%%%%%%
global flag;%%%%%%%% Flag set to '1' to plot graph for different pwer levels  %%%%%%%%%%%
n=1;

%%%%%%%%%%%%%%%%--------Variable declaration-----------%%%%%%%%%%%%%%%%%
no_of_nodes = 300;
no_of_clusters = 2;
power_vector=[14 12 5 3 1 -1 -5]; %%%%%% (in dBm) %%%%%%%
transmitPower=14;
rx_sensitivity = -89; %%%%%% (in dBm) %%%%%%%
frequency = 2400; %%%%%% (in MHz) %%%%%%

%%%%%%%%%%%%%%%%%%%----Create random coordinates-----%%%%%%%%%%%%%%%%%%%%%
rng shuffle;
rng ('default')
x_cooordinates = randperm(10000,no_of_nodes);
y_cooordinates = randperm(10000,no_of_nodes);
coordinates = [x_cooordinates' y_cooordinates'];
index_node=find(coordinates(:,1));
coordinate_index=horzcat(index_node,coordinates);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%******************************************%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%---------CALL USER DEFINED FUNCTIONS-----------%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%------------Create Random Nodes--------------%%%%%%%%%%%%%%%
CreateRandomNodes(x_cooordinates, y_cooordinates);

%%%%%%%%%%%%%---------Call function Create k-means---------%%%%%%%%%%%%%%%
[new_centroid_coordinates,orphan_coordinates,intracluster_distance,no_of_clusters]=CreateK_Means(no_of_clusters,coordinates,...
    no_of_nodes,rx_sensitivity,x_cooordinates,y_cooordinates,power_vector,frequency);
power_matrix=zeros(no_of_clusters,1);
power_matrix(:,1)=14;

node_to_cluster_connectivity_matrix=calculate_node_to_cluster_connectivity_matrix(new_centroid_coordinates,...
    coordinates,transmitPower,rx_sensitivity,no_of_clusters,no_of_nodes);

[common_nodes,transpose_connectivity_matrix,backbone_graph] = CalculateCommonNodesBetweenClusters(...
    node_to_cluster_connectivity_matrix,no_of_clusters,no_of_nodes);

[common_nodes,updated_power_matrix,node_to_cluster_connectivity_matrix]=OptimizeNetwork(new_centroid_coordinates,...
    coordinates,no_of_clusters,power_vector,no_of_nodes,rx_sensitivity,frequency,power_matrix,x_cooordinates,y_cooordinates,...
node_to_cluster_connectivity_matrix,common_nodes);

track_matrix=zeros(no_of_nodes,3);
for i=1:no_of_clusters
    for j=1:no_of_nodes
        if(node_to_cluster_connectivity_matrix(i,j)==1)
            track_matrix(j,1)=coordinate_index(j,1);
            track_matrix(j,2)=updated_power_matrix(i);
            track_matrix(j,3)=i;
        end
    end
end


[shortest_path_matrix,cm,cluster_power_consumption,hop_matrix,transpose_shortest_path_matrix, no_of_hops_count,...
    percentage_of_messages_delivered]=PlotShortestPathGraph(common_nodes,no_of_clusters,no_of_nodes,track_matrix,updated_power_matrix);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%****************************************************%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%-------- FUNCTION DEFINITION---------%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%-----Function to create random 300 nodes-----%%%%%%%%%%%%%%%%
function CreateRandomNodes(x_cooordinates, y_cooordinates)
a = 0; %m
b = 10000; %m
hold on;
axis([a b a b]);
figure(1);
plot(x_cooordinates,y_cooordinates,'bO');
hold on;
grid on;
grid on, xlabel('x'), ylabel('y');
xlabel('x-coordinates');
ylabel('y-coordinates');
title('300 Random nodes'); 
end

%%%%%%%%%%%%------------Function for CreateK_Means()-----------%%%%%%%%%%%%%%
%%%%%-------Get Optimum centroids & Cluster Index for each node------%%%%%%
%%%%%%%%%%%---------Get Original Centroid Coordinates---------%%%%%%%%%%%%%
function [new_centroid_coordinates,orphan_coordinates,centroid_to_node_distance,no_of_clusters]=CreateK_Means(no_of_clusters,coordinates,no_of_nodes,...
    rx_sensitivity,x_cooordinates,y_cooordinates,transmitPower,frequency)
%%%%%%%%%%%%%%Global Variables%%%%%%%%%%%%%%%%%%
global n;
global flag;

%%%%%Use k-means to get optimum centroids & cluster index for each node%%%%
[node_cluster_index,centroid_coordinates]=kmeans(coordinates,no_of_clusters);

%%%%%%%%%%%%%%Call function OriginalCentroidCoordinates()%%%%%%%%%%%%%%%%%%
new_centroid_coordinates=OriginalCentroidCoordinates(no_of_clusters,no_of_nodes,node_cluster_index,coordinates,centroid_coordinates);

%%%%%%%%%%%%%%%%% Declaration of variables %%%%%%%%%%%%%%%%%%%%
original_clusters=no_of_clusters;
original_min_centroid=zeros(length(original_clusters),2);
original_min_centroid=new_centroid_coordinates(:,:);
%max_distance=zeros(1,length(power_vector));

%%%%%%%%%%%%%-----Iteration for different power levels-------%%%%%%%%%%%%%%%
%for transmitPower=1:length(power_vector)
transmitPower=14;
    %%%%%%%%%%%%% Reassiging of values for different power levels %%%%%%%%%%%
    new_centroid_coordinates=zeros(length(no_of_clusters),2);
    no_of_clusters=original_clusters;
    new_centroid_coordinates=original_min_centroid(:,:);
    n=1;
    flag=0;
    %%%%%%%%%%% Call CalculateRadiusWithDifferentPower() to calculate %%%%%%%%%%%
    %%%%%%%%%%% maximum range a cluster head can receive from its nodes %%%%%%%%%%%
   % max_distance(transmitPower)=CalculateRadiusWithDifferentPower(rx_sensitivity,frequency,power_vector(transmitPower));
    max_distance =CalculateRadiusWithDifferentPower(rx_sensitivity,frequency,transmitPower);
    %%%%%%%%%%%%%% Call NodeCoverage() for different power levels %%%%%%%%%%%%%%%  
   % [centroid_to_node_distance,orphan_coordinates,no_of_clusters,new_centroid_coordinates]=NodeCoverage(new_centroid_coordinates,coordinates,...
      %  no_of_clusters,no_of_nodes,rx_sensitivity,power_vector(transmitPower),x_cooordinates,y_cooordinates,max_distance(transmitPower));
    [centroid_to_node_distance,orphan_coordinates,no_of_clusters,new_centroid_coordinates]=NodeCoverage(new_centroid_coordinates,coordinates,...
        no_of_clusters,no_of_nodes,rx_sensitivity,transmitPower,x_cooordinates,y_cooordinates,max_distance);
%end

end


%%%%%%%%%%%%%%%--------Get nearest node as a centroid--------%%%%%%%%%%%%%%
%%%%%%%%%%%%--------Using optimum centroids & all 300 nodes-----%%%%%%%%%%%
function new_centroid_coordinates=OriginalCentroidCoordinates(no_of_clusters,no_of_nodes,node_cluster_index,coordinates,centroid_coordinates)
new_centroid_coordinates=zeros(no_of_clusters,2);
for cluster_no=1:no_of_clusters
    min_dist=15000;
    for nodes=1:no_of_nodes
        if(nodes~=no_of_nodes)
            if(node_cluster_index(nodes)==cluster_no)
                node_distance=sqrt(((coordinates(nodes,1)-centroid_coordinates(cluster_no,1))^2)+((coordinates(nodes,2)-centroid_coordinates(cluster_no,2))^2));
                if(node_distance<min_dist)
                    min_dist=node_distance;
                    new_centroid_coordinates(cluster_no,:)=coordinates(nodes,:);
                end
            end
        end
    end
end
end

%%%%%%%%%%%%%%%%%%%%%-----*******-----MOST IMPORTANT FUNCTION----*******-----%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%-----All the nodes are covered with different powers----%%%%%%%%%%
%%%%%%%%%------Node Coverage function is called recursively-----%%%%%%%%%%%
function [centroid_to_node_distance,orphan_coordinates,no_of_clusters,new_centroid_coordinates]=NodeCoverage(new_centroid_coordinates,...
    coordinates,no_of_clusters,no_of_nodes,rx_sensitivity,transmitPower,x_cooordinates,y_cooordinates,max_distance)

global n;
global flag;

%%%%%%%%%%%%%%%%%%% Declaration of local variables %%%%%%%%%%%%%%%%%%%%%
a = 0; %m
b = 10000; %m
fspl_matrix=zeros(no_of_clusters,no_of_nodes);
margin_matrix=zeros(no_of_clusters,no_of_nodes);
rx_power_matrix=zeros(no_of_clusters,no_of_nodes);
centroid_to_node_distance=zeros(no_of_clusters,no_of_nodes);


%%%%%%%%%%%%%%% Call CalculateNodesDistanceFromCentroid() to calculate nodes distance from centroid %%%%%%%%%%%%%%%%%%%
dist=CalculateNodesDistanceFromCentroid(new_centroid_coordinates,coordinates,no_of_clusters,no_of_nodes);

%%%%%%%%%%%%%%%%%%%%%% Calculate the rx power matrix %%%%%%%%%%%%%%%%%%%%%%
for i=1:no_of_clusters
    for j=1:no_of_nodes
        %%%%%%%%%%%% Check if distance matrix has any value 0 %%%%%%%%%%%%%
        %%%%%%%%%%%%%% If its 0, do not consider, else FSPL matrix will have infinite value %%%%%%%%%%%%%% 
        if(dist(i,j)~=0) 
            fspl_matrix(i,j)=36.56+20*log10(2400)+20*log10((dist(i,j)/1000.0)/1.609);
            margin_matrix(i,j)=transmitPower+abs(rx_sensitivity)-fspl_matrix(i,j);
            rx_power_matrix(i,j)=margin_matrix(i,j)-abs(rx_sensitivity);
            if(rx_power_matrix(i,j) > -89)    
                centroid_to_node_distance(i,j)=(dist(i,j)/1000)/1.609;
            end
        end      
    end
end

%%%%%%%%%%%%%%%% Calculate orphan coordinates for each centroid %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% Gets updated after every iteration %%%%%%%%%%%%%%%%%%%%%
orphan_coordinates=GetOrphanCoordinates(no_of_clusters,no_of_nodes,centroid_to_node_distance,new_centroid_coordinates,coordinates);
% new_centroid_coordinates=[new_centroid_coordinates;zeros(1,2)];

if(n==(no_of_nodes+1))  
%     new_centroid_coordinates=[new_centroid_coordinates;zeros(1,2)];
      return;
end

%%%%%%%%%%%%%%% Loop for calling function recursively to eliminate nodes outside cluster %%%%%%%%%%%%% 
while(n<(no_of_nodes+1))
    if(orphan_coordinates(n,:)>0)    
        new_centroid_coordinates(no_of_clusters+1,:)=orphan_coordinates(n,:); 
        no_of_clusters=no_of_clusters+1;
        if(n<(no_of_nodes+1))
        [centroid_to_node_distance,orphan_coordinates,no_of_clusters,new_centroid_coordinates]=NodeCoverage(new_centroid_coordinates,...
            coordinates,no_of_clusters,no_of_nodes,rx_sensitivity,transmitPower,x_cooordinates,y_cooordinates,max_distance);
        end
    else
        n=n+1;
    end
end

%%%%%%%%%%%%%%%%%% Plot clusters for different Power Level %%%%%%%%%%%%%%%%%%%
if(flag==0)
    
%%%%%%%%%%%% Calculate Power consumption For the Entire Network using different Power levels for all the nodes %%%%%%%%%%%%%%%
dBm_to_mW=10^(transmitPower/10);
power_consumed = dBm_to_mW*no_of_clusters;

radius3=max_distance*ones(length(new_centroid_coordinates),1);
f=[];
f= strcat(num2str(no_of_clusters),' clusters',' at Power Level ',{' '},num2str(transmitPower),'dBm');
hold on;
figure;
viscircles(new_centroid_coordinates,radius3,'LineWidth',1);
hold on;
axis([a b a b]);
hold on;
grid on;
grid on, xlabel('x'), ylabel('y');
xlabel('x-coordinates')
ylabel('y-coordinates')
plot(x_cooordinates,y_cooordinates,'b*');
hold on;
plot(new_centroid_coordinates(:,1),new_centroid_coordinates(:,2), 'm+');
title(f); 
flag=1;

end 

end

%%%%%%%%%%%%%%---- Function to calculate nodes distance from all the centroid ----%%%%%%%%%%%%%
function dist=CalculateNodesDistanceFromCentroid(new_centroid_coordinates, coordinates,no_of_clusters,no_of_nodes)
dist=zeros(no_of_clusters,no_of_nodes);
for i=1:no_of_clusters
    for j=1:no_of_nodes
        dist(i,j)=sqrt(((coordinates(j,1)-new_centroid_coordinates(i,1))^2)+((coordinates(j,2)-new_centroid_coordinates(i,2))^2));
    end
end
end

%%%%%%%%%%%%%--------- Function to get the orphan coordinates ----------%%%%%%%%%%%%%
function orphan_coordinates=GetOrphanCoordinates(no_of_clusters,no_of_nodes,centroid_to_node_distance,new_centroid_coordinates,coordinates)
orphan_coordinates=zeros(no_of_nodes,2);
for i=1:no_of_clusters
    for j=1:no_of_nodes
        if(centroid_to_node_distance(:,j)==0)
            orphan_coordinates(j,:)=coordinates(j,:);                     
        end
    end
    
end
%%%%%%%%%%%%%% Check if orphan coordinates are not centroids %%%%%%%%%%%%%%
%%%%%%%%%% centroid_to_node_distance will be 0 if a node is a centroid %%%%%%%%%%%
for i=1:no_of_clusters
    for j=1:no_of_nodes
        if(orphan_coordinates(j,:)==new_centroid_coordinates(i,:)) 
              orphan_coordinates(j,:)=0;
              orphan_coordinates(j,:)=0;
        end 
    end
end
end

%%%%%%%%%%%%%%%%%%%----Radius using different Power----%%%%%%%%%%%%%%%%%%%%
function max_distance=CalculateRadiusWithDifferentPower(rx_sensitivity,...
    frequency,transmitPower)
rx_power=rx_sensitivity;
max_distance=10^((abs(rx_power)+transmitPower-36.56 - (20*log10(frequency)))/20);
max_distance = (1.609 * max_distance)*1000;
end

%%%%%%%%%-----Check that all the nodes are covered by clusters----%%%%%%%%
function node_to_cluster_connectivity_matrix=calculate_node_to_cluster_connectivity_matrix(new_centroid_coordinates, coordinates,transmitPower,rx_sensitivity,...
no_of_clusters,no_of_nodes)
node_to_cluster_connectivity_matrix=zeros(no_of_clusters,no_of_nodes);

%%%%%%%%%%%%%%%%%% Calcuate distance between %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% centroids and all the nodes %%%%%%%%%%%%%%%%%%%%
dist=CalculateNodesDistanceFromCentroid(new_centroid_coordinates,coordinates,no_of_clusters,no_of_nodes);

%%%%%%%%%%%%%%%%%%%%%% Calculate the rx power matrix %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% Create node and cluster connectivity matrix %%%%%%%%%%%%%%%
for i=1:no_of_clusters
    for j=1:no_of_nodes
        fspl_matrix(i,j)=36.56+20*log10(2400)+20*log10((dist(i,j)/1000.0)/1.609);
        margin_matrix(i,j)=transmitPower+abs(rx_sensitivity)-fspl_matrix(i,j);
        rx_power_matrix(i,j)=margin_matrix(i,j)-abs(rx_sensitivity);
        if(rx_power_matrix(i,j) > -89)    
          node_to_cluster_connectivity_matrix(i,j)=1;
        end     
    end
end

count =0;
%%%%%% Check if all nodes are covered by clusters %%%%%%%%%% 
%%%%%%% If all nodes are covered, then count should be 300 %%%%%%%%
for i=1:no_of_nodes
    if(max(node_to_cluster_connectivity_matrix(:,i))==1)
        count = count+1;
    end   
end
end

%%%%%%%%%% Function to calculate common nodes between two or mre clusters %%%%%%%%%
function [common_nodes,transpose_connectivity_matrix,backbone_graph] = CalculateCommonNodesBetweenClusters(...
    node_to_cluster_connectivity_matrix,no_of_clusters,no_of_nodes)
common_nodes = zeros(no_of_clusters);
transpose_connectivity_matrix=transpose(node_to_cluster_connectivity_matrix);
for i=1:no_of_nodes
    for j=1:no_of_clusters
        for k=1:no_of_clusters
            if(transpose_connectivity_matrix(i,j)==1 && transpose_connectivity_matrix(i,k)==1 && j~=k)
                common_nodes(j,k)=common_nodes(j,k)+1;
            end
        end
    end
end
%%%%%%%%%%% Plot backbone graph when Power level is 14 dBm %%%%%%%%%%%%%
backbone_graph=graph(common_nodes);
figure;
hold on;
grid on;
plot(backbone_graph,'NodeColor','r','EdgeLabel',backbone_graph.Edges.Weight,'LineWidth',1);
hold on;
title('Backnone Graph when Power level is 14dbm');
end

%%%%%%%%%%%%%% Plot shortest path graph for 10000transanctions %%%%%%%%%%%%
function [shortest_path_matrix,cm,cluster_power_consumption,hop_matrix,transpose_shortest_path_matrix,no_of_hops_count,...
    percentage_of_messages_delivered]=PlotShortestPathGraph(common_nodes,no_of_clusters,no_of_nodes,track_matrix,updated_power_matrix)

%%%%%%%%%%%% Backbone graph when clusters have different poer level %%%%%%%%%%
G=graph(common_nodes);
figure;
hold on;
grid on;
plot(G,'NodeColor','r','EdgeLabel',G.Edges.Weight,'LineWidth',1);
hold on;
title('Backnone Graph for Optimized Network');

%%%%%% Using connected componect functio to check if network is broken %%%%%%%
%%%%%%%%% should always give '1' for all the cluster nodes %%%%%%%%%
cm=conncomp(G);
[r_size,c_size]=size(G.Edges);

%%%%%%%% To make the 'NUMBER OF HOPS' as the cost for shortest Path, set %%%%%%%
%%%%%%%% Edge weight to '1' (Previously th edge weights are defined by %%%%%%%%
%%%%%%%% number of common nodes in a network %%%%%%%%%%%%%
for i=1:r_size
    G.Edges.Weight(i)=1;
end

shortest_path_matrix=zeros(no_of_clusters);
from_node=zeros(1,no_of_nodes);
to_node=zeros(1,no_of_nodes);
from=zeros(1,10000);
to=zeros(1,10000);

%%%%%%% Loop for getting random nodes as source and destination nodes %%%%%%%
%%%%%%% Random nodes should not be zero %%%%%%%
%%%%%%% Source and destination nodes cannot be same %%%%%%%
for ti=1:10000
    from_node(ti)=randi(no_of_nodes);
    to_node(ti)= randi(no_of_nodes);
    while (from_node(ti)==0)
        from_node(ti)=randi(no_of_nodes);
    end
    while(from_node(ti)==to_node(ti)|| to_node(ti)==0) 
         to_node(ti)= randi(no_of_nodes);
    end 
end

%%%%%%%%% Loop for identifying the clusters for a random generated nodes %%%%%%%
%%%%%%%%% as number of hops between source and destination is from cluster %%%%%%%%%
%%%%%%%%%%%%%%%%%% to cluster %%%%%%%%%%%%%%%%%%%%%
for vi=1:10000
    for d=1:no_of_nodes
        if(from_node(vi)==track_matrix(d,1))
            from(vi)=track_matrix(d,3);
        end
        if(to_node(vi)==track_matrix(d,1))
            to(vi)=track_matrix(d,3);
        end
    end
end

%%%%%%%%% Hop matrix stores 'number of hops' %%%%%%%%%
%%%%%%%%% taken to  go from source to destination %%%%%%%%%
%%%%%%%%% If source and destination are within same cluster %%%%%%%
%%%%%%%%% Then number of hops is 1 %%%%%%%%%%%
hop_matrix=zeros(10000,1);
for s=1:10000
    
    %%%%%% shortetpath() returns the shortest path followed from source to %%%%%%
    %%%%%%% destination where number of hops is the minimum cost taken %%%%%%%%
     [shortest_path,no_of_hops]=shortestpath(G,from(s),to(s));
     if(no_of_hops==0)
         no_of_hops=1;
     end
     hop_matrix(s)=no_of_hops;
     [row,col]=size(shortest_path);
     for t=1:col
        shortest_path_matrix(s,t)=shortest_path(t);  
     end
transpose_shortest_path_matrix=transpose(shortest_path_matrix);
end

%%%%%%%% count the number of times a paricular hop takes place %%%%%%%%%
no_of_hops_count=zeros(max(hop_matrix),1);
for n=1:10000
    for m=1:max(hop_matrix)
        if(hop_matrix(n)==m)
            no_of_hops_count(m)=no_of_hops_count(m)+1;
        end
    end
end

%%%%%%% Gives the percentage of messages delivered at a particular hop %%%%%%%
%%%%%%%%%%%% (no_of_hops_count./10000)*100= no_of_hops_count./100 %%%%%%%%%%%%
percentage_of_messages_delivered=zeros(max(hop_matrix),1);
percentage_of_messages_delivered=no_of_hops_count./100;

%%%%%%%%%% Graph for Percentage of message delivered %%%%%%%%%%
%%%%%%%%% successfully with respect to number of hops %%%%%%%%%%
x=1:max(hop_matrix);
hold on;
figure;
hold on;
grid on;
grid on,xlabel('x'), ylabel('y');
plot(x,percentage_of_messages_delivered);
xlabel('Number of Hops');
ylabel('Percentage of Message Delivered')
hold off;

%%%%%%%%%% Number of times a particular cluster has been used %%%%%%%%%%
%%%%%%%% during 10000 transanctions between source and destination %%%%%%%%
count_clusters=zeros(no_of_clusters,1);
for r=1:10000
    for rt=1:no_of_clusters
        for nc=1:no_of_clusters
            if(shortest_path_matrix(r,rt)==nc & shortest_path_matrix(r,rt)~=0)
                count_clusters(nc,1)=count_clusters(nc,1)+1;
            end
        end
    end
end

%%%%%%%%%%%%%%%%%% Convert power from dBm to mW for each cluster %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%% Computer the power consumed by each cluster %%%%%%%%%%%%%%%%%%%
%%%%%%%% cluster_power_consumption(sb,1)= dBm_to_mW(sb,1)*count_clusters(sb,1); %%%%%%%

dBm_to_mW=zeros(no_of_clusters,1);
cluster_power_consumption=zeros(no_of_clusters,1);
for sb=1:no_of_clusters
    dBm_to_mW(sb,1)=10^((updated_power_matrix(sb,1))/10);
    cluster_power_consumption(sb,1)= dBm_to_mW(sb,1)*count_clusters(sb,1);
end

%%%%%%%%% Graph obtained for Power Consumption for each cluster %%%%%%%%%
x=1:no_of_clusters;
figure;
bar(x,cluster_power_consumption);
hold on;
grid on;
grid on, xlabel('Number of Clusters'), ylabel('Power Consumtion(*10^4 mW)');
end

%%%%%%%%%% Function to optimize  the network with different power levels %%%%%%%%%%%
function [common_nodes,updated_power_matrix,node_to_cluster_connectivity_matrix]=OptimizeNetwork(new_centroid_coordinates,...
    coordinates,no_of_clusters,power_vector,no_of_nodes,rx_sensitivity,frequency,power_matrix,x_cooordinates,y_cooordinates,...
node_to_cluster_connectivity_matrix,common_nodes)

%%%%%% calculate distance between centroids & nodes %%%%%%%
dist=CalculateNodesDistanceFromCentroid(new_centroid_coordinates,coordinates,no_of_clusters,no_of_nodes);

%%%%%% Track connectivity matrix & common nodes matrix %%%%%%%
track_connectivity_matrix=zeros(no_of_clusters,no_of_nodes);
track_common_nodes=zeros(no_of_clusters);
 for i=1:no_of_clusters 
     for j=1:no_of_nodes
        track_connectivity_matrix(i,j)=node_to_cluster_connectivity_matrix(i,j);
     end
 end
 
 for i=1:no_of_clusters
     track_common_nodes(i,:)=common_nodes(i,:);
 end
 
 %%%%%%%% Track power matrix %%%%%%%%%
updated_power_matrix=zeros(no_of_clusters,1);
track_power_matrix=zeros(no_of_clusters,1);
for i=1:no_of_clusters
    updated_power_matrix(i,1)=power_matrix(i,1);
end

%%%%%%%% Loop Over Power %%%%%%
for p=2:length(power_vector)
    %%%%%%%%% Calculate Radius wrt power %%%%%%%%
    max_distance=CalculateRadiusWithDifferentPower(rx_sensitivity,frequency,power_vector(p));
    
    %%%%%%%% Loop over clusters %%%%%%%%
    for i=1:no_of_clusters 
        
        %%%%%%%%% Updated common nodes and power matrix with selected power %%%%%%
        track_power_matrix(i,1)=updated_power_matrix(i,1);
        updated_power_matrix(i,1)=power_vector(p);
        
        %%%%%%%% Loop over nodes %%%%%%%%
           for j=1:no_of_nodes
               track_connectivity_matrix(i,j)=node_to_cluster_connectivity_matrix(i,j);
%                %%%%%%% Check if distance matrix is less than equal to %%%%%
%                %%%%%%%%%%%%%%%%%%% Maximum distance %%%%%%%%%%%%%%%%%%%%%

                if(dist(i,j)<=max_distance) 
                    node_to_cluster_connectivity_matrix(i,j)=1;
                else
                    node_to_cluster_connectivity_matrix(i,j)=0;
                end      
           end
           
           %%%%%% Check if all the nodes are covered by the clusters %%%%%%
           %%%%%% return count # of nodes (300)  when all nodes are covered %%%%%%%%%
            count=0;
            for t=1:no_of_nodes
                if(max(node_to_cluster_connectivity_matrix(:,t))==1)
                    count = count+1;
                end 
            end
            
            %%%%%%%% Keep traack for common nodes %%%%%%%%%%%%%%%
            for c=1:no_of_clusters
                    for k=1:no_of_clusters
                        track_common_nodes(c,k)=common_nodes(c,k);
                    end
            end
            
            %%%%%%%%%%%%%% Check for common nodes for updated bode and cluster connectivity matrix %%%%%%%%%%%%%%%%
            common_nodes = zeros(no_of_clusters);
            transpose_connectivity_matrix=transpose(node_to_cluster_connectivity_matrix);
            for nt=1:no_of_nodes
                for c=1:no_of_clusters
                    for k=1:no_of_clusters
                        if(transpose_connectivity_matrix(nt,c)==1 && transpose_connectivity_matrix(nt,k)==1 && c~=k)
                                common_nodes(c,k)=common_nodes(c,k)+1;
                        end
                    end
                end
            end
     
           %%%%%%%% Check if there are any clusters which does not have %%%%%%%
           %%%%%%%% common nodes, return non-zero value %%%%%%%%
           %%%%%%%%%% and return 0 if there are common nodes in a cluster %%%%%%%%
            count_chk=0;
            for jy=1:no_of_clusters
                if(sum(common_nodes(jy,:))==0)
                    count_chk = count_chk+1;
                end 
            end
            
            %%%%% Update power matrix, node to cluster connectivitity
            %%%%% matrix & common nodes if all nodes are covered and all
            %%%%% lusters have some common nodes with other network in a
            %%%%% cluster else update to previous state. %%%%%%%%%%%
            if(count~=no_of_nodes || count_chk~=0)
                updated_power_matrix(i,1)=track_power_matrix(i,1); 
                 node_to_cluster_connectivity_matrix(i,:)=track_connectivity_matrix(i,:);
                 for c=1:no_of_clusters
                    for k=1:no_of_clusters
                        common_nodes(c,k)=track_common_nodes(c,k);
                    end
                 end
            else
                %%%%%%% check if the graph obtained is connected %%%%%%%%%%%%
                %%%%%%% If it is not then return back to previous state %%%%%%%
                %%%%%%% else update power matrix, common nodes and %%%%%%%%%%%%
                %%%%%%%%%%%%% node_t_cluster_connectivity matrix %%%%%%%%%%%%%%%
                 G=graph(common_nodes);
                 conn_comp=conncomp(G);
                 comp_count=0;
                 for cl=1:no_of_clusters
                     if(conn_comp(1,cl)~=1)
                         comp_count= comp_count+1;
                     end
                 end
                 if(comp_count~=0)
                     updated_power_matrix(i,1)=track_power_matrix(i,1); 
                     node_to_cluster_connectivity_matrix(i,:)=track_connectivity_matrix(i,:);
                     for c=1:no_of_clusters
                       for k=1:no_of_clusters
                            common_nodes(c,k)=track_common_nodes(c,k);
                       end
                    end
                 else
                     track_power_matrix(i,1)=updated_power_matrix(i,1);
                     track_connectivity_matrix(i,:)=node_to_cluster_connectivity_matrix(i,:);
                     for c=1:no_of_clusters
                       for k=1:no_of_clusters
                            track_common_nodes(c,k)=common_nodes(c,k);
                       end
                    end
                 end
             end
    end
end

%%%%%%% Get radius for different power level %%%%%%%%
radius=zeros(no_of_clusters,1);
for ct=1:no_of_clusters
    radius(ct,1)=CalculateRadiusWithDifferentPower(rx_sensitivity,frequency,updated_power_matrix(ct,1));
end
   
a = 0; %m
b = 10000; %m

%%%%%%%%% Draw circle around each centroid with their respective radius %%%%%%%%%
radius3=radius.*ones(length(no_of_clusters),1);
f=[];
f= strcat(num2str(no_of_clusters),' clusters',' at different Power Levels');
hold on;
figure; 
viscircles(new_centroid_coordinates,radius3);
hold on;
axis([a b a b]);
hold on;
grid on;
grid on, xlabel('x'), ylabel('y');
plot(x_cooordinates,y_cooordinates,'b*');
hold on;
xlabel('x-coordinates');
ylabel('y-coordinates')
hold on;
plot(new_centroid_coordinates(:,1),new_centroid_coordinates(:,2), 'm+');
hold on;
title(f);
end
