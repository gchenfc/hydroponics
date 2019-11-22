import paramiko

def init():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(hostname='raspberrypi.local', username='pi', key_filename='/home/gerry/.ssh/github.pub')
    return ssh

def takeImage(ssh, filename, debug=True):
    stdin, stdout, stderr = ssh.exec_command('raspistill -o %s' % filename)
    if (debug):
        for line in stdout.read().splitlines():
            print(line)

def exit(ssh):
    ssh.close()

def main():
    ssh = init()
    takeImage(ssh, 'Desktop/test.jpg')
    exit(ssh)

if __name__ == '__main__':
    main()