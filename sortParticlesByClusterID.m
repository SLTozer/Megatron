function [cIDs,weights,particles] = sortParticlesByClusterID(cIDs,weights,particles)

    sortedMat = sortrows([cIDs ; weights ; particles]',1)'

    cIDs       = sortedMat(1,:)
    weights    = sortedMat(2,:)
    particles  = sortedMat(3,:)

end
