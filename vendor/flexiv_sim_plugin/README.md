# Vendored Flexiv Sim Plugin payload

This directory contains the binary SDK payload required by the native Flexiv
fleet transport. It is a build dependency, not the home of fleet application
logic. Fleet orchestration lives in `scripts/controllers`, and the native transport
lives in `native/flexiv_async_fleet`.

The Flexiv headers and static archive came from `flexivrobotics/flexiv_sim_plugin`
commit `6ed94b66fa076c473a7564efc6da2452588a5dc2`. The exact static dependency
payload used to link that archive is included so this workspace can build
without a second repository checkout.

Dependency revisions:

- Fast DDS: `33e1d9cd5bf9c142c95d545c6ce45727ff599076`
- Fast CDR: `7ebc45e03e1dd010681f5594135edbdcf3556c50`
- foonathan_memory_vendor: `e91681a25711c1811b2eaf2ba1e6996e0d9ecc84`
- spdlog: `27cb4c76708608465c413f6d0e6b8d99a4d84302`
- tinyxml2: `bf15233ad88390461f6ab0dbcf046cce643c5fcb`

Licenses are retained in this directory and `thirdparty/licenses`.
