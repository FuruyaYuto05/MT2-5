#include <Novice.h>
#include <imgui.h>
#include <cmath>
#include <algorithm>

const char kWindowTitle[] = "AABB Collision Example";

// ---------------- Vector3 ----------------
struct Vector3 {
	float x, y, z;
};

Vector3 Add(const Vector3& a, const Vector3& b) {
	return { a.x + b.x, a.y + b.y, a.z + b.z };
}
Vector3 Subtract(const Vector3& a, const Vector3& b) {
	return { a.x - b.x, a.y - b.y, a.z - b.z };
}
Vector3 Multiply(float s, const Vector3& v) {
	return { v.x * s, v.y * s, v.z * s };
}

// ---------------- Vector Math ----------------
float Dot(const Vector3& a, const Vector3& b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}
Vector3 Cross(const Vector3& a, const Vector3& b) {
	return {
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x
	};
}
float Length(const Vector3& v) {
	return sqrtf(Dot(v, v));
}
Vector3 Normalize(const Vector3& v) {
	float len = Length(v);
	if (len == 0) return { 0, 0, 0 };
	return Multiply(1.0f / len, v);
}

// ---------------- Matrix4x4 ----------------
struct Matrix4x4 {
	float m[4][4];
};

Matrix4x4 MakeIdentity() {
	Matrix4x4 m{};
	for (int i = 0; i < 4; i++) m.m[i][i] = 1.0f;
	return m;
}
Matrix4x4 MakeViewMatrix(Vector3 eye, Vector3 target, Vector3 up) {
	Vector3 zaxis = Normalize(Subtract(target, eye));
	Vector3 xaxis = Normalize(Cross(up, zaxis));
	Vector3 yaxis = Cross(zaxis, xaxis);

	Matrix4x4 m{};
	m.m[0][0] = xaxis.x; m.m[1][0] = xaxis.y; m.m[2][0] = xaxis.z;
	m.m[0][1] = yaxis.x; m.m[1][1] = yaxis.y; m.m[2][1] = yaxis.z;
	m.m[0][2] = zaxis.x; m.m[1][2] = zaxis.y; m.m[2][2] = zaxis.z;
	m.m[3][0] = -Dot(xaxis, eye);
	m.m[3][1] = -Dot(yaxis, eye);
	m.m[3][2] = -Dot(zaxis, eye);
	m.m[3][3] = 1.0f;
	return m;
}
Matrix4x4 MakePerspectiveFovMatrix(float fovY, float aspect, float nearZ, float farZ) {
	Matrix4x4 m{};
	float f = 1.0f / tanf(fovY / 2.0f);
	m.m[0][0] = f / aspect;
	m.m[1][1] = f;
	m.m[2][2] = farZ / (farZ - nearZ);
	m.m[2][3] = 1.0f;
	m.m[3][2] = -nearZ * farZ / (farZ - nearZ);
	return m;
}
Matrix4x4 Multiply(const Matrix4x4& a, const Matrix4x4& b) {
	Matrix4x4 result{};
	for (int y = 0; y < 4; y++)
		for (int x = 0; x < 4; x++)
			for (int k = 0; k < 4; k++)
				result.m[y][x] += a.m[y][k] * b.m[k][x];
	return result;
}
Vector3 Transform(const Vector3& v, const Matrix4x4& m) {
	float x = v.x * m.m[0][0] + v.y * m.m[1][0] + v.z * m.m[2][0] + m.m[3][0];
	float y = v.x * m.m[0][1] + v.y * m.m[1][1] + v.z * m.m[2][1] + m.m[3][1];
	float z = v.x * m.m[0][2] + v.y * m.m[1][2] + v.z * m.m[2][2] + m.m[3][2];
	float w = v.x * m.m[0][3] + v.y * m.m[1][3] + v.z * m.m[2][3] + m.m[3][3];
	if (w != 0.0f) { x /= w; y /= w; z /= w; }
	return { x, y, z };
}

// ---------------- Screen Projection ----------------
Vector3 ProjectToScreen(const Vector3& v) {
	return { v.x * 640 + 640, -v.y * 360 + 360, 0 };
}

// ---------------- AABB Struct & Collision ----------------
struct AABB {
	Vector3 min;
	Vector3 max;
};

bool IsAABBCollision(const AABB& a, const AABB& b) {
	return (a.min.x <= b.max.x && a.max.x >= b.min.x) &&
		(a.min.y <= b.max.y && a.max.y >= b.min.y) &&
		(a.min.z <= b.max.z && a.max.z >= b.min.z);
}

void FixAABBMinMax(AABB& box) {
	for (int i = 0; i < 3; i++) {
		float& minRef = ((&box.min.x)[i]);
		float& maxRef = ((&box.max.x)[i]);
		if (minRef > maxRef) std::swap(minRef, maxRef);
	}
}

// ---------------- Draw AABB ----------------
void DrawAABB(const AABB& box, const Matrix4x4& viewProj, int color /* ARGB */) {
	Vector3 corners[8] = {
		{box.min.x, box.min.y, box.min.z},
		{box.max.x, box.min.y, box.min.z},
		{box.max.x, box.max.y, box.min.z},
		{box.min.x, box.max.y, box.min.z},
		{box.min.x, box.min.y, box.max.z},
		{box.max.x, box.min.y, box.max.z},
		{box.max.x, box.max.y, box.max.z},
		{box.min.x, box.max.y, box.max.z},
	};

	int edges[12][2] = {
		{0,1}, {1,2}, {2,3}, {3,0},
		{4,5}, {5,6}, {6,7}, {7,4},
		{0,4}, {1,5}, {2,6}, {3,7}
	};

	for (int i = 0; i < 12; i++) {
		Vector3 p0 = ProjectToScreen(Transform(corners[edges[i][0]], viewProj));
		Vector3 p1 = ProjectToScreen(Transform(corners[edges[i][1]], viewProj));
		Novice::DrawLine((int)p0.x, (int)p0.y, (int)p1.x, (int)p1.y, color);
	}
}

// ---------------- Main ----------------
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {
	Novice::Initialize(kWindowTitle, 1280, 720);

	AABB box1 = { {-0.5f, -0.5f, -0.5f}, {0.0f, 0.0f, 0.0f} };
	AABB box2 = { {0.2f, 0.2f, 0.2f}, {1.0f, 1.0f, 1.0f} };

	float cameraAngle = 0.5f;
	float cameraDistance = 200.0f;

	while (Novice::ProcessMessage() == 0) {
		Novice::BeginFrame();

		ImGui::Begin("AABB Control");
		ImGui::DragFloat3("Box1 Min", &box1.min.x, 1.0f);
		ImGui::DragFloat3("Box1 Max", &box1.max.x, 1.0f);
		ImGui::DragFloat3("Box2 Min", &box2.min.x, 1.0f);
		ImGui::DragFloat3("Box2 Max", &box2.max.x, 1.0f);
		ImGui::SliderFloat("Camera Angle", &cameraAngle, 0.0f, 6.28f);
		ImGui::SliderFloat("Camera Distance", &cameraDistance, 10.0f, 500.0f);
		ImGui::End();

		// 入れ替えチェック
		FixAABBMinMax(box1);
		FixAABBMinMax(box2);

		// カメラ設定
		Vector3 eye = { sinf(cameraAngle) * cameraDistance, cameraDistance * 0.5f, cosf(cameraAngle) * cameraDistance };
		Vector3 target = { 0, 0, 0 };
		Vector3 up = { 0, 1, 0 };

		Matrix4x4 view = MakeViewMatrix(eye, target, up);
		Matrix4x4 proj = MakePerspectiveFovMatrix(0.45f, 1280.0f / 720.0f, 0.1f, 1000.0f);
		Matrix4x4 viewProj = Multiply(view, proj);

		// 衝突判定
		bool collision = IsAABBCollision(box1, box2);

		// 描画
		DrawAABB(box1, viewProj, 0xFFFFFFFF); // 白
		DrawAABB(box2, viewProj, collision ? 0xFF0000FF : 0xFFFFFFFF); // 赤または白

		Novice::EndFrame();
	}

	Novice::Finalize();
	return 0;
}
