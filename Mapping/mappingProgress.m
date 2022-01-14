function mapping_progress = mappingProgress(occMap, voxels)
    isOccupied = checkOccupancy(occMap, voxels);
    num_voxels = length(isOccupied);
    num_voxels_free = sum(isOccupied == 0);
    num_voxels_occ = sum(isOccupied == 1);
    num_voxels_unm = sum(isOccupied == -1);
    mapping_progress = [num_voxels_free num_voxels_occ num_voxels_unm] / num_voxels;
end

