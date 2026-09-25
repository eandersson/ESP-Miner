import { TestBed } from '@angular/core/testing';

import { GithubUpdateService } from './github-update.service';
import { provideHttpClient, withXhr } from '@angular/common/http';

describe('GithubUpdateService', () => {
  let service: GithubUpdateService;

  beforeEach(() => {
    TestBed.configureTestingModule({
      providers: [provideHttpClient(withXhr())]
    });
    service = TestBed.inject(GithubUpdateService);
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });
});
