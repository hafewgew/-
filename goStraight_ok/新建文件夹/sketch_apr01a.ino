
float prevYaw = 0;     // 上一次的角度值
float prevX = 0;       // 上一次的 X 轴坐标
float prevY = 0;       // 上一次的 Y 轴坐标
float distance = 0;    // 行走距离
float x = 0;           // 当前 X 轴坐标
float y = 0;           // 当前 Y 轴坐标
float yaw = 0;         // 当前角度值

void  zuobiao(){
// 获取当前角度值
yaw = mpu_getyaw();

// 计算行走距离
float deltaDistance = (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2 - distance;
distance += deltaDistance;
// 计算当前 X 和 Y 坐标
x += deltaDistance * cos(radians(yaw));
y += deltaDistance * sin(radians(yaw));

// 计算当前方向
float deltaAngle = yaw - prevYaw;
if (deltaAngle > 180) {
  deltaAngle -= 360;
}
else if (deltaAngle < -180) {
  deltaAngle += 360;
}
float direction = prevYaw + deltaAngle / 2;
if (direction < 0) {
  direction += 360;
}
else if (direction >= 360) {
  direction -= 360;
}

// 更新上一次的坐标和角度值
prevX = x;
prevY = y;
prevYaw = yaw;

float dx = x - prevX;
float dy = y - prevY;
float heading = radians(direction);
float d = sqrt(dx * dx + dy * dy);
float deltaHeading = atan2(dy, dx);
float deltaTheta = deltaHeading - heading;
float R = d / deltaTheta;

// 计算当前位置和方向
x = prevX + R * sin(heading + deltaTheta);
y = prevY - R * cos(heading + deltaTheta);
direction = direction + degrees(deltaTheta);

// 更新上一次的坐标和角度值
prevX = x;
prevY = y;
prevYaw = yaw;


}
