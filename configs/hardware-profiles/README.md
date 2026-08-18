# Hardware safety profiles

将额外的 B2 真机安全配置以 `.json` 文件放在本目录。控制中心会自动发现，并在
“Hardware Activation Gate”下拉框中允许选择其中一套；不需要修改 Python 或 C++
代码。

每个文件使用与 `../b2-reference-profile.json` 相同的 schema。配置之间互相独立，
但每套配置仍必须完整、标记为 `approved`、允许激活，并与所选 ONNX policy 的
SHA-256 匹配，才能启动真机 LowCmd gateway。文件路径必须位于本目录，网页请求不能
传入任意外部路径。
