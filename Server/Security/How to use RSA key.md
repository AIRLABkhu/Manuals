## 한국어
### 환경
- Ubuntu 18.04
### SSH를 위한 RSA 보안 매뉴얼
#### 1. Linux 서버에서 RSA 공개키-개인키 쌍을 만든다.
   ```console
   $ ssh-keygen -m rsa -p pem
   $ # RSA 키 이름을 지정한다. 기본값: ~/.ssh/id_rsa
   $ # 키가 만들어진다. (개인키: id_rsa, 공개키: id_rsa.pub)
   ```
#### 2. 공개키를 인증 키로 등록한다.
   ```console
   $ cat ~/.ssh/{PRI_KEY_NAME} > ~/.ssh/authorized_keys
   ```
#### 3. 개인키를 클라이언트에 전송한다. 
   * 서버가 클라이언트로 전송하는 경우
     ```console
     $ cd ~/.ssh
     $ scp -P {CLIENT_PORT} {PRI_KEY_NAME} {User}@{IP}:./
     ```
   * 클라이언트가 서버로부터 가져오는 경우
     ```console
     $ cd ~/.ssh
     $ scp -P {SERVER_PORT} {User}@{IP}:~/.ssh/{PRI_KEY_NAME} ./
     ```
#### 4. RSA 키를 사용해 서버에 접속한다.
  * CMD 또는 Terminal을 사용해 접속하는 경우
    ```console
    $ ssh {User}@{IP} -p {PORT} -i {KEY_FULLPATH}
    ```
  * Visual Studio Code를 사용해 접속하는 경우
    *SSH 접속 설정 파일 e.g. ~/.ssh/config*
    ```c
    Host AIRLAB_Library
        User {User}
        HostName {IP}
        Port {Port}
        IdentityFile "{Key Fullpath}"  # Recommend using doublequote.
    ```
#### 5. 클라이언트에서 중요 파일을 생성하고 권한을 설정한다.
  ```console
  touch ~/.ssh/known_hosts

  chmod 700 ~/.ssh                  # 유저 전용, 모든 권한
  chmod 600 ~/.ssh/{PRI_KEY_NAME}   # 유저 전용, 읽기 및 쓰기
  chmod 644 ~/.ssh/{PUB_KEY_NAME}   # 모든 유저, 읽기 전용, 소유자만 쓰기 가능
  chmod 600 ~/.ssh/authorized_keys  # 유저 전용, 읽기 및 쓰기
  chmod 644 ~/.ssh/known_hosts      # 모든 유저, 읽기 전용, 소유자만 쓰기 가능
  ```
#### 6. 해당 계정에 대해 비밀번호 접속을 차단한다. (선택사항, 권장사항)
  1. SSH 설정 파일을 연다.
        ```console
        $ vi /etc/ssh/sshd_config   # 또는
        $ nano /etc/ssh/sshd_config
        ```
  2. 'PasswordAuthentication' 설정을 찾고 아래 코드로 변경한다.
        ```ini
        PasswordAuthentication yes
        Match User {User1}, {User2}, ..., {UserN}
            PasswordAuthentication no
        Match all
        ```
  3. 위 구문이 이미 있다면 ```{UserN}``` 에 해당 계정을 추가한다.
  4. 접속 중인 유저가 있는지 확인한 후 SSH daemon을 재시작한다. 
        ```console
        $ service ssh restart   # 또는
        $ systemctl restart sshd
        ```

## English
### Environment
- Ubuntu 18.04
### Manual of RSA Security for SSH
#### 1. Generate RSA publickey-privatekey pair on your Linux server.
   ```console
   $ ssh-keygen -m rsa -p pem
   $ # Specify your key name. Default: ~/.ssh/id_rsa
   $ # The keys will be generated. (Public key: id_rsa, Private key: id_rsa.pub)
   ```
#### 2. Register the key as an authorized key.
   ```console
   $ cat ~/.ssh/{PRI_KEY_NAME} > ~/.ssh/authorized_keys
   ```
#### 3. Send the private key to your client.
   * The server sends to the client.
     ```console
     $ cd ~/.ssh
     $ scp -P {CLIENT_PORT} {PRI_KEY_NAME} {User}@{IP}:./
     ```
   * The client acquires from the server.
     ```console
     $ cd ~/.ssh
     $ scp -P {SERVER_PORT} {User}@{IP}:~/.ssh/{PRI_KEY_NAME} ./
     ```
#### 4. Access the server using RSA key.
  * Using CMD or Terminal.
    ```console
    $ ssh {User}@{IP} -p {PORT} -i {KEY_FULLPATH}
    ```
  * Using Visual Studio Code
    *SSH configuration file e.g. ~/.ssh/config*
    ```c
    Host AIRLAB_Library
        User {User}
        HostName {IP}
        Port {Port}
        IdentityFile "{Key Fullpath}"  # Recommend using doublequote.
    ```
#### 5. Generate some files and set privileges.
  ```console
  touch ~/.ssh/known_hosts

  chmod 700 ~/.ssh                  # User only, All privileges
  chmod 600 ~/.ssh/{PRI_KEY_NAME}   # User only, Read & Write
  chmod 644 ~/.ssh/{PUB_KEY_NAME}   # All users, Read-only, Only the owner can write
  chmod 600 ~/.ssh/authorized_keys  # User only, Read & Write
  chmod 644 ~/.ssh/known_hosts      # All users, Read-only, Only the owner can write
  ```
#### 6. Block password authentication for the user. (Optional, Recommended)
  1. Open SSH configuration file.
        ```console
        $ vi /etc/ssh/sshd_config   # or
        $ nano /etc/ssh/sshd_config
        ```
  2. Find 'PasswordAuthentication' and replace with below.
        ```ini
        PasswordAuthentication yes
        Match User {User1}, {User2}, ..., {UserN}
            PasswordAuthentication no
        Match all
        ```
  3. If the syntax exists, append the user name as ```{UserN}```.
  4. Restart sshd daemon. Make sure no one is using ssh.
        ```console
        $ service ssh restart   # or
        $ systemctl restart sshd
        ```
