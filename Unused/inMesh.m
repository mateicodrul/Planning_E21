function in = inMesh(fv, points)
% Returns if a point is inside a closed, connected triangulated surface mesh
% Author: Ander Biguri
maxZ = max(fv.vertices(:,3));
counts = zeros(size(points,1),1);
for ii = 1:size(points,1)
    ray = [points(ii,:);points(ii,1:2) maxZ+1];
    for jj = 1:size(fv.faces,1)
        v = fv.vertices(fv.faces(jj,:),:);
        if all(v(:,3) < ray(1,3))
            continue;
        end
        isin = mollerTrumbore(ray, fv.vertices(fv.faces(jj,:),:));
        counts(ii) = counts(ii)+isin;
    end
end
in = mod(counts,2);
end