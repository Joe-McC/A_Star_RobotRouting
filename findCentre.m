function mapCentre = findCentre(mappedPoints)
centreNodes=[];
count=0;
for i=7:10
    for j=7:10
        if mappedPoints(i,j)==2
            %check if neighbour equals 2, if not it is an anomoly
            if mappedPoints(i+1,j)||mappedPoints(i-1,j)||mappedPoints(i,j+1)||mappedPoints(i,j-1)
                newRow=[i,j];
                centreNodes=[centreNodes;newRow];
                count=count+1;
            end
        end
    end
end
totalX=0;
totalY=0;
for c=1:count
    totalX=totalX+centreNodes(c,1);
    totalY=totalY+centreNodes(c,2);
end
centreX=totalX/count;
centreY=totalX/count;
mapCentre=[centreX,centreY];
end
