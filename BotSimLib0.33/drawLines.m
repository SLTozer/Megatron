function [] = drawLines( lines,col )
%drawLines draws a line
for i = 1:size(lines,1)
    line(lines(i,[1 3]),lines(i,[2 4]),'lineWidth',1,'color',col); % draws scanLines
end
end

