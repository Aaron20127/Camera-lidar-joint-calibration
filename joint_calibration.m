%%
main()

function main()
    close all;
    %% pointcloud file
    % Polywork2019 fitting calibration plate corner file, .igs format
    curdir = pwd;
    igsFileDir = [curdir '/' 'data/chessboard_pointcloud_igs/'];
    % pointcloud file, only have pointcloud xyz, .txt format 
    pointcloudFileDir = [curdir '/' 'data/pointcloud/'];
    % chessboard grids
    x_grids = 5; % the number of grids on the short side
    y_grids = 8; % the number of grids on the long side 

    % image point
    imageDir = 'data/images/';
    imageType = 'png';
    MinCornerMetric = 0.4; % Adjusting this parameter can better detect corner points

    % only show result of image corner detection
    onlyShowDetection = 'false';

    % camera parameters
    focalLength    = [2525.9, 2528.1]; % fx, fy
    principalPoint = [942.9102, 584.8342]; % cx, cy
    imageSize = [1080, 1920];

    % LM solve pnp
    global M; % camera intrinsic matrix
    M = [focalLength(1), 0,  principalPoint(1); 0, focalLength(2), principalPoint(2); 0, 0, 1];
    
    % run
    [imageFileNames, chessboardPointcloudFileNames] = getFileNameFromDir(imageDir, igsFileDir, imageType);
    pointcloud = getChessboardPointcloudFromIgs(chessboardPointcloudFileNames, x_grids, y_grids);
    imagePoints = getImagePoints(imageFileNames, MinCornerMetric, onlyShowDetection, x_grids, y_grids);

    if ~strcmp(onlyShowDetection, 'true')
        [R,T] = LM_solvePnP(imagePoints, pointcloud, focalLength, principalPoint, imageSize);
        showReprojectImageError(imagePoints, pointcloud, imageFileNames, R, T, ...
                                focalLength, principalPoint);
        showPointCloudToImage(imageFileNames, R, T, focalLength, principalPoint, pointcloudFileDir);
    end

end

% LM sovle PnP
function [R,T] = LM_solvePnP(imagePoints, worldPoints, focalLength, principalPoint, imageSize)
    % Connect the points of all images to shape of Nx2
    jointImagePoints = [];
    for i = 1:numel(imagePoints)
        jointImagePoints = [jointImagePoints; imagePoints{i}];
    end

    % Connect the points of all point cloud to shape of Nx2
    jointWorldPoints = [];
    for i = 1:numel(worldPoints)
        jointWorldPoints = [jointWorldPoints; worldPoints{i}];
    end

    % init, very important, we can use estimateWorldCameraPose() to initialize R, T
    intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);
    [R0, T0] = estimateWorldCameraPose(jointImagePoints, jointWorldPoints, ...
                intrinsics, 'Confidence', 99.99, 'MaxReprojectionError', 6);
    x0 = [R0(1,:),R0(2,:),R0(3,:),T0];
    xdata = jointWorldPoints;
    ydata = [jointImagePoints; [0,0;0,0;0,0;1,1;1,1;1,1]]; % add orthogonal matrix constraint

    % LM algorithm
    options = optimoptions('lsqcurvefit','Algorithm','levenberg-marquardt');
    lb = [];
    ub = [];    
    options.MaxFunctionEvaluations = inf;
    options.MaxIterations = inf;
    options.FunctionTolerance = 1.0000e-8;
    options.StepTolerance = 1.0000e-8;

    [x,resnorm,residual,exitflag,output] = lsqcurvefit(@reProjection,x0,xdata,ydata,lb, ub,options)
    
    R = reshape(x(1:9),3,3)';
    detR = det(R)
    R
    T = x(10:12)
end

% reProjection
function value=reProjection(x, xdata)
    global M;
    R = reshape(x(1:9),3,3)';
    T = x(10:12);

    cameraPoints = R * (xdata' - T');
    perspectivePoints = M * cameraPoints;
    uvs = perspectivePoints ./ perspectivePoints(end,:);
    value = uvs(1:end-1,:)';
    
    % add orthogonal matrix constraint
    Rl = [dot(R(1,:),R(2,:)), dot(R(1,:), R(3,:)); ...
          dot(R(2,:),R(3,:)), dot(R(:,1), R(:,2)); ...
          dot(R(:,1),R(:,3)), dot(R(:,2), R(:,3)); ...
          norm(R(1,:)),  norm(R(2,:)); ...
          norm(R(3,:)),  norm(R(:,1)); ...
          norm(R(:,2)),  norm(R(:,3))]
           
    value = [value; Rl];
end

% get image file path and pointcloud file path
function [imageFileNames, chessboardPointcloudFileNames] = getFileNameFromDir(imageDir, igsFileDir, imageType)
    imageFileNames = {};
    chessboardPointcloudFileNames = {};

    dirOutput = dir(fullfile(imageDir,['*.' imageType]));
    imageFileBaseNames = {dirOutput.name};
    for i=1:numel(imageFileBaseNames)
        [pathstr,name,suffix] = fileparts(imageFileBaseNames{i});
        igsFileName = [igsFileDir '/' name '.igs'];
        if 2 == exist(igsFileName, 'file')
            imageFileNames{i} = [imageDir '/' imageFileBaseNames{i}];
            chessboardPointcloudFileNames{i} = igsFileName;
        end
    end
end

% get image corner of chessboard
function imagePoints = getImagePoints(imageFileNames, MinCornerMetric, showDetection, x_grids, y_grids) 
    % Detect checkerboards in images
    [imagePointsLists, boardSize, imagesUsed] = ...
        detectCheckerboardPoints(imageFileNames, 'MinCornerMetric', MinCornerMetric);
    
    imageFileNames = imageFileNames(imagesUsed);
    
    % cell format
    imagePoints = {};
    for i = 1:size(imagePointsLists,3)
        %imagePoints{i} = imagePointsLists(:,:,i);
        imagePoints{i} = reorderImageCornerPoints(imagePointsLists(:, :, i), x_grids, y_grids);
    end

    % show chessboard image points
    if strcmp(showDetection, 'true')
        for i = 1:numel(imageFileNames)
            % draw chessboard corner points
            I = imread(imageFileNames{i});
            figure('NumberTitle', 'off', 'Name', ['detection ' imageFileNames{i}])
            imshow(I);
            hold on;
            plot(imagePoints{i}(1,1),imagePoints{i}(1,2),'go','MarkerSize', 8);       
            hold on;
            plot(imagePoints{i}(2:end,1),imagePoints{i}(2:end,2),'ro','MarkerSize', 8);
        end
    end
end

% Let the minmum point of y be the origin
function imageCornerPoints = reorderImageCornerPoints(input, x_grids, y_grids)
    % corner of largist y
    [row, col] = size(input);
    four_corner_row = [1, x_grids-1, row - (x_grids-1)+1, row];
    four_corner_y = [input(four_corner_row(1),2); ...
                     input(four_corner_row(2),2); ...
                     input(four_corner_row(3),2); ...
                     input(four_corner_row(4),2)];
    [value, index] = sort(four_corner_y, 'ascend');
    
    % reorder
    point = [];
    min_y_row = four_corner_row(index(1));
    if min_y_row == four_corner_row(2)
        for i = 1:(y_grids-1)
           for j = 1:(x_grids-1)
               r = (i-1) * (x_grids-1) + (x_grids-1) - j + 1;
               point = [point; input(r,:)];
           end
        end
    elseif  min_y_row == four_corner_row(3)
        for i = 1:(y_grids-1)
           r =  ((y_grids-1) - i) * (x_grids-1);
           point = [point; input(r+1:r+(x_grids-1),:)];
        end
    elseif  min_y_row == four_corner_row(4)
        for i = 1:row
            point = [point; input(row-i+1,:)]
        end
    else
        point = input;
    end

    imageCornerPoints = point
end

% get pointcloud and Let the maxmum point of z be the origin
function pointcloud = getChessboardPointcloudFromIgs(fileNamesPath, x_grids, y_grids)
    % get four pointcloud
    four_pointcloud = {};
    for i = 1:numel(fileNamesPath)
        % copy file, change filename
        [pathstr,name,suffix] = fileparts(fileNamesPath{i});
        txt_file_path = [pathstr '/' name '.txt'];
        copyfile(fileNamesPath{i},txt_file_path);

        % read txt file
        cell_text = readcell(txt_file_path);

        % get points
        p1 = strsplit(cell_text{11},',');
        p2 = strsplit(cell_text{12},',');
        p3 = strsplit(cell_text{13},',');
        points1 = [str2num(p1{1}), str2num(p1{2}), str2num(p1{3})];
        points2 = [str2num(p1{4}), str2num(p2{1}), str2num(p2{2})];
        points3 = [str2num(p2{3}), str2num(p2{4}), str2num(p3{1})];
        points4 = [str2num(p3{2}), str2num(p3{3}), str2num(p3{4})];
        chessboard = [points1;points2;points3;points4];

        % sort z from largest to smallest
        [value, index] = sort(chessboard(:,end), 'descend');
        descend_points = [chessboard(index(1),:); chessboard(index(2),:); chessboard(index(3),:);chessboard(index(4),:)];
        four_pointcloud{i} = descend_points;
    end

    % fit chessboard pointcloud
    pointcloud = getChessboardGridPointcloud(four_pointcloud, x_grids, y_grids);
end

% fitting chessboard pointcloud
function pointcloud = getChessboardGridPointcloud(four_pointcloud, x_grids, y_grids)
    % fit chessboard pointcloud
    pointcloud = {};

    for i = 1:numel(four_pointcloud)
        p = four_pointcloud{i};

        origin = p(1,:); 
        vec_x = p(2,:) - p(1,:); % vector of x-axis
        vec_y = p(3,:) - p(1,:); % vector of y-axis

        points_cloud_grids = [];
        for y = 1:(y_grids-1)
            for x = 1:(x_grids-1)
                new_points = origin + vec_x * (x/x_grids) + vec_y * (y/y_grids);
                points_cloud_grids = [points_cloud_grids; new_points];
            end
        end
        
        pointcloud{i} = points_cloud_grids;
    end
end


% reprojection chessboard points
function showReprojectImageError(imagePoints, pointcloud, imageFileNames, R, T, ...
    focalLength, principalPoint)
    for i = 1:numel(imageFileNames)
        % draw chessboard corner points
        I = imread(imageFileNames{i});
        figure('NumberTitle', 'off', 'Name', ['reProjection ' imageFileNames{i}])
        imshow(I);
        hold on;
        plot(imagePoints{i}(1,1),imagePoints{i}(1,2),'go','MarkerSize', 8);       
        hold on;
        plot(imagePoints{i}(2:end,1),imagePoints{i}(2:end,2),'ro','MarkerSize', 8);

        % draw reprojection points
        global M;
        cameraPoints = R * (pointcloud{i}' - T');
        perspectivePoints = M * cameraPoints;
        uvs = perspectivePoints ./ perspectivePoints(end,:);
        uv = uvs(1:end-1,:)';

        hold on;
        plot(uv(1,1),uv(1,2),'gx','MarkerSize', 8);
        hold on;
        plot(uv(2:end,1),uv(2:end,2),'bx','MarkerSize', 8);
    end
end

% reprojection all pointcloud
function showPointCloudToImage(imageFileNames, R, T, ...
                               focalLength, principalPoint, pointCloudPath)
    global M;
    for i = 1:numel(imageFileNames)
        [pathstr,name,suffix]=fileparts(imageFileNames{i});
        pointCloudFilePath = [pointCloudPath '/' name '.txt'];
        if 2 == exist(pointCloudFilePath, 'file')
            % show image
            I = imread(imageFileNames{i});
            figure('NumberTitle', 'off', 'Name', ['PointCloudToImage ' imageFileNames{i}])
            imshow(I);

            % get points
            pointCloud = [];
            pointCloudCell = readcell(pointCloudFilePath);
            for i = 1:size(pointCloudCell, 1)
                point = [pointCloudCell{i,1}, pointCloudCell{i,2}, pointCloudCell{i,3}];
                pointCloud = [pointCloud; point];
            end

            % projection
            cameraPoints = R * (pointCloud' - T');
            perspectivePoints = M * cameraPoints;
            uvs = perspectivePoints ./ perspectivePoints(end,:);
            uv = uvs(1:end-1,:)';

            % draw points
            hold on;
            plot(uv(1:1,1),uv(1:2,2),'o','MarkerSize', 3, 'MarkerEdgeColor', 'g');
            plot(uv(2:end,1),uv(2:end,2),'o','MarkerSize', 3, 'MarkerEdgeColor', 'b');
        end
    end
end