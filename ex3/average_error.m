% Computes the average Euclidean distance between A and B.
% Assumes both A and B are M*N matrices. Computes the Euclidean distance
% between respective column vectors in A and B (i.e., M-dim vectors),
% and returns the average over the N distances.

function err = average_error(A, B)
    assert(all(size(A) == size(B)));
    err = mean(sqrt(sum((A - B).^2)));
end